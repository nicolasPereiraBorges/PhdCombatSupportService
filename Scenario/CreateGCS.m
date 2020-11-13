function gcs = CreateGCS()
%REATEGCS Summary of this function goes here
%   Detailed explanation goes here

gcs = GroundControlStation();
gcs.Id = 1;
gcs.Position = Position3D(1,1,0);
end

