function map = add_obstacles(obs, map, varargin)
% ADDOBSTACLESFROMXYLIST Add fixed-shape obstacles to a binaryOccupancyMap
%   map = addObstaclesFromXYList(obs, map) adds default circular obstacles
%   centered at each (x,y) in obs(:,1:2). obs(:,3) is ignored.
%
%   Optional parameters:
%     'Shape'         - 'circle' (default) or 'rectangle'
%     'Radius'        - Radius for circles (default: 0.5)
%     'HalfWidth'     - Half-width for rectangles (default: 0.5)
%     'HalfHeight'    - Half-height for rectangles (default: 0.3)
%     'Inflate'       - Inflation radius (default: 0)

  p = inputParser;
  addRequired(p, 'obs', @(x) isnumeric(x) && size(x,2) >= 2);
  addRequired(p, 'map', @(m) isa(m, 'binaryOccupancyMap'));
  addParameter(p, 'Shape', 'circle', @(x) ischar(x) || isstring(x));
  addParameter(p, 'Radius', 0.5, @isscalar);
  addParameter(p, 'HalfWidth', 0.5, @isscalar);
  addParameter(p, 'HalfHeight', 0.3, @isscalar);
  addParameter(p, 'Inflate', 0, @isscalar);
  parse(p, obs, map, varargin{:});
  ops = p.Results;

  for i = 1:size(obs,1)
    xi = obs(i,1);
    yi = obs(i,2);
    switch lower(ops.Shape)
      case 'circle'
        theta = linspace(0, 2*pi, 60);
        cx = xi + ops.Radius*cos(theta);
        cy = yi + ops.Radius*sin(theta);
        pts = [cx(:), cy(:)];
        setOccupancy(map, pts, true);
      case 'rectangle'
        w = ops.HalfWidth;
        h = ops.HalfHeight;
        nx = ceil(2*w * map.Resolution);
        ny = ceil(2*h * map.Resolution);
        [gx, gy] = meshgrid(linspace(xi-w, xi+w, nx), ...
                            linspace(yi-h, yi+h, ny));
        pts = [gx(:), gy(:)];
        setOccupancy(map, pts, true);
      otherwise
        error('Unknown shape: %s', ops.Shape);
    end
  end

  if ops.Inflate > 0
    inflate(map, ops.Inflate);
  end
end
