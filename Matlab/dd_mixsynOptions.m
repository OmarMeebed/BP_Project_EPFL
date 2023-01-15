function parameters = dd_mixsynOptions(varargin)
%
% dd_mixsynOptions(name1,value1, name2, value2, ...)
% Structure of parameters using in dd_mixsyn.
%
% --------------------------------------------------------------------
%     name          ->          description
% --------------------------------------------------------------------
% 'K'               -> intitial stabilizing controller
% 'order'           -> order of the final controller (≥ order 'K')
% 'force_integrator'-> forces an integrator in the final controller
% 'modulus_margin'  -> add a constraint to the modulus margin
% 'W'               -> Frequency grid where the problem is solved
% 'norm'            -> solver for either the 'two' or 'inf' norm
% 'max_iteration'   -> number of iteration before stopping
% 'tol'             -> stops if decrese in objective is less than tol
% --------------------------------------------------------------------
%
%Example use:
%options = dd_mixsynOptions('K',K, 'order', orderK, ...
%                           'W',W, ...
%                           'modulus_margin',1/2,'force_integrator',true, ...
%                           'max_iteration',20, 'tol', 1e-4,...
%                           'norm','two');

parameters = struct('K',[], 'order',[],'force_integrator',false,'modulus_margin', [], 'W',[],'norm','inf' ,'max_iteration',10,'tol',0);

tokens = {};

if mod(nargin,2) ~= 0
    error('arguments must come in pairs');
end

for ii = 1 : 2: nargin
    key  = varargin{ii};
    value= varargin{ii+1};
    switch lower(deblank(key))
        case 'order'
            tokens{end+1} = struct('key', key,'value',value);
        otherwise
            if isfield(parameters,key)
                parameters.(key) = value;
            else
                warning('Parameters %s not recoginosed',key)
            end
    end
end


for token = [tokens{:}]
    % only process theses values after the rest has been set
    switch lower(token.key)
        case 'order'
            try
                [a,b,Ts] = tfdata(parameters.K,'v');
            catch
                error('Controller should be a state-space, zpk, or transfer function with correct sampling time')
            end
            if length(a)-1 < token.value
                
                a(token.value+1) = 0;
                b(token.value+1) = 0;
                parameters.K = tf(a,b,Ts);
                parameters.order = token.value;
            end
            if length(a)-1 > token.value
            error('Initial controller can not be of higher order than final controller')
            end
    end
end



end