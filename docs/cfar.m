DATADIR = '~/dart/data';
DATASET = 'path/to/dataset';
TRAIN_PCT = 0.8;

radarfile = fullfile(DATADIR, DATASET, 'radar.h5');
trajfile = fullfile(DATADIR, DATASET, 'trajectory.h5');
mask = h5read(trajfile, '/mask');
t = h5read(trajfile, '/t');
pos = h5read(trajfile, '/pos').';
vel = h5read(trajfile, '/vel').';
rot = permute(h5read(trajfile, '/rot'), [3 2 1]);
rad = permute(h5read(radarfile, '/rad'), [4 3 2 1]);
rad = rad(strcmp(mask, 'TRUE'), :, :, :);

sensorfile = fullfile(DATADIR, DATASET, 'sensor.json');
fid = fopen(sensorfile);
raw = fread(fid, inf);
fclose(fid);
sensor = jsondecode(char(raw'));
sensor.a = [-1; 1-2/8; 8];

GUARD_BAND = 5;
TRAIN_BAND = 10;
CFAR = 1e-2;

AZ_UPSAMPLE_FACTOR = 100;

cfar2d = phased.CFARDetector2D(...
	'GuardBandSize', GUARD_BAND, ...
	'TrainingBandSize', TRAIN_BAND, ...
	'ProbabilityFalseAlarm', CFAR, ...
	'OutputFormat', 'Detection index' ...
);

imin = GUARD_BAND + TRAIN_BAND;
[nframes, rres, dres, ares] = size(rad);
[didx, ridx] = meshgrid(1+imin:dres-imin, 1+imin:rres-imin);
cidx = [ridx(:) didx(:)].';

dt = mean(diff(t));
allpoints = [];
for i = 1:nframes*TRAIN_PCT
	frame = squeeze(rms(rad(i, :, :, :), 4));
	detections = cfar2d(frame, cidx);
	if norm(vel(i,:)) > 0.2
		points = zeros(size(detections, 2), 4);
		for j = 1:size(detections, 2)
			r = detections(1, j);
			d = detections(2, j);
			det = squeeze(rad(i, r, d, :));
			a = azimuth_subpixel(det, AZ_UPSAMPLE_FACTOR);
			xyz = rda2xyz(r, d, a, pos(i,:), vel(i,:), squeeze(rot(i,:,:)), sensor);
			gain = xyz2gain(xyz, pos(i,:), squeeze(rot(i,:,:)));
			if all(isreal(xyz)) && isreal(gain) && gain > 0
				points(j,:) = [xyz, frame(r, d) ./ gain];
			end
		end
		allpoints = [allpoints; points(points(:,4) > 0, :)];
	end
	i/(nframes*TRAIN_PCT)
end

outfile = fullfile(DATADIR, DATASET, 'cfar.h5');
h5create(outfile, '/pos', [size(allpoints, 1) 3]);
h5create(outfile, '/amplitude', size(allpoints, 1));
h5write(outfile, '/pos', allpoints(:,1:3));
h5write(outfile, '/amplitude', allpoints(:,4));


function xyz = rda2xyz(r, d, a, pos, vel, rot, sensor)
    rval = sensor.r(1) + (r-1) ./ (sensor.r(3)-1) .* (sensor.r(2) - sensor.r(1));
    dval = sensor.d(1) + (d-1) ./ (sensor.d(3)-1) .* (sensor.d(2) - sensor.d(1));
    aval = sensor.a(1) + (a-1) ./ (sensor.a(3)-1) .* (sensor.a(2) - sensor.a(1));
    az = -asin(aval);

    s = norm(vel);
    v = vel * rot ./ s;
    dnorm = dval ./ s;
    B = v(1).*cos(az) + v(2).*sin(az);
    C = v(3);
    D = -dnorm;
    z = (C.*D + B.*sqrt(B.^2 - D.^2 + C.^2)) ./ (B.^2 + C.^2);
    z(z>1) = (C.*D - B.*sqrt(B.^2 - D.^2 + C.^2)) ./ (B.^2 + C.^2);
    z(~isreal(z)) = (C.*D - B.*sqrt(B.^2 - D.^2 + C.^2)) ./ (B.^2 + C.^2);
    y = sin(az).*sqrt(1-z.^2);
    x = sqrt(1-y.^2-z.^2);

    xyz = rval * [x, y, z] * rot.' + pos;
end

function a = azimuth_subpixel(det, upsample_factor)
    N = length(det);
    f = linspace(-1, 1, upsample_factor * N);
    a = ones(N, 1);
    n = 0:N-1;
    b = abs(exp(-1j * pi * n.' * f).' * a) / N;
    lobes = b / rms(b);
    udet = upsample(det, upsample_factor) / rms(det);
    r = cconv(udet, fliplr(lobes), upsample_factor * N);
    [~, imax] = max(r);
    a = mod(imax / upsample_factor + N/2, N) + 1;
end

function gain = awr1843gain(theta, phi)
    theta_ = theta / pi * 180 / 56;
    phi_ = phi / pi * 180 / 56;
    gaindb = 0.14*theta_.^6 + 0.13*theta_.^4 - 8.2*theta_.^2 ...
           + 3.1*phi_.^8 - 22*phi_.^6 + 54*phi_.^4 - 55*phi_.^2;
    gain = 10.^(gaindb/20);
end

function gain = xyz2gain(xyz, pos, rot)
    xyz_sensor = (xyz - pos) * rot;
    r = norm(xyz_sensor);
    el = asin(xyz_sensor(3) ./ r);
    az = asin(xyz_sensor(2) ./ (r .* cos(el)));
    gain = awr1843gain(el, az) ./ r.^2;
end
