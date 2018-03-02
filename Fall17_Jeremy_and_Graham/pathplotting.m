%%% X_Y path plotting
function [] = pathplotting ()


function [x,y] = math_convert(latitude, longitude)

	rad_earth = 6371000;
	%Hardcoded origin
	origin = [deg2rad(42.44814), deg2rad(-76.48489)];
	latitude = deg2rad(latitude);
	longitude = deg2rad(longitude);
	a = (sin((latitude-origin(0))/(2.0)))^2 + cos(latitude)*cos(origin(0))*(sin((longitude-origin(1))/(2.0)))^2;
	theta = 2*arctan2(sqrt(a), sqrt(1-a));
	d = theta * rad_earth;
	bearing = bearing_from_origin(origin, latitude, longitude);
	%# print "HERE", d
	x = d*sin(bearing); 
    y = d*cos(bearing);
end


    function [bearing] = bearing_from_origin(origin, latitude, longitude)
        y = sin(longitude-origin(1))*cos(latitude)
        x = cos(origin(0))*sin(latitude) - sin(origin(0))*cos(latitude)*cos(longitude-origin(1));
        %Bearing in radians
        bearing = arctan2(y, x);
        %Bearing in degrees
        bearing_deg = rad2deg(bearing);
        %print "BEARING", bearing_deg
    end




end

	