function s = v2struct(varargin)
for i = 1:nargin
    s.(inputname(i))=varargin(i);
end
end