% figureid = orEnvPlot(points,...)
%
% plots points or lines in the openrave viewer
% points - Nx3 vector of xyz positions
% optional arguments include 'size', 'color', and 'line'
%   color - Nx3 vector of RGB values between 0 and 1
%   size - Nx1 vector of the sizes in pixels of each point/line
%   line (or linestrip) - if specified, then openrave renders a line strip
%   linelist - if specified, openrave renders a line for every two points
%   trilist - if specified, openrave renders a triangle for every three
%             vertices should be specified in counter-clockwise order
%   sphere - if specified, openrave renders each point as a sphere
%   transparency - [0,1], set transparency of plotted objects (0 is opaque)
function figureid = orEnvPlot(varargin)

session = openraveros_getglobalsession();
req = openraveros_env_plot();

points = varargin{1}';
numpoints = size(points,2);
req.points = points(:);

req.size = 0.5;
req.drawtype = req.Draw_Point();
req.transparency = 0;

i = 2;
while(i <= nargin)
    if( strcmp(varargin{i},'color') )
        i = i + 1;
        colors = varargin{i}';
        req.colors = colors(:);
    elseif( strcmp(varargin{i},'size') )
        i = i + 1;
        req.size = varargin{i};
    elseif( strcmp(varargin{i},'line') | strcmp(varargin{i},'linestrip') )
        req.drawtype = req.Draw_LineStrip();
    elseif( strcmp(varargin{i},'linelist') )
        req.drawtype = req.Draw_LineList();
    elseif( strcmp(varargin{i},'sphere') )
        req.drawtype = req.Draw_Sphere();
    elseif( strcmp(varargin{i},'trilist') )
        req.drawtype = req.Draw_TriList();
    elseif( strcmp(varargin{i},'transparency') )
        i = i + 1;
        req.transparency = varargin{i};
    end
    
    i = i + 1;
end

res = rosoct_session_call(session.id,'env_plot',req);

if(~isempty(res))
    figureid = res.figureid;
else
    figureid = [];
end
