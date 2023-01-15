function varargout = dd_mixsyn(varargin)
%
%dd_mixsyn Data-driven mixed-sensitivity controller design
%Computes the optimal controller for the mixed synthesis problem.
%For the feedback loop
%
%                     e       u
%            r --->O--->[ K ]-->[ G ]---+---> y
%                - |                    |
%                  +<-------------------+
%
%    the mixed-sensitivity design seeks a controller K that minimizes
%
%                   ||  W1*S  ||
%             GAM = || W2*K*S ||
%                   ||  W3*T  ||_norm
%    where
%      * S = inv(I+G*K) is the sensitivity function
%      * K*S is the transfer from r to u (control effort)
%      * T = I-S = G*K/(I+G*K) is the complementary sensitivity
%      * W1,W2,W3 are weights chosen to emphasize certain frequency bands.
%        For example, to enforce low sensitivity and good tracking at low
%        frequency, W1 should have high gain at low frequency and low gain
%        at high frequency.
%      * norm is the norm specified in options (either 'two' or 'inf')
%
% [K,CL,GAM,INFO] = dd_mixsyn(G,W1,W2,W3,options)
%
%If multi-model uncertainity is used, use G = cat(1,G1,G2,..);
%
%Example use:
%options = dd_mixsynOptions(..)
%W1 = ...
%W2 = ...
%W3 = ...
%[K,CL,gam] = dd_mixsyn(G,W1,W2,W3,options);
%
G = varargin{1};

opt = varargin{end};
if ~isa(opt,'struct')
    error('Last argument should be a dd_mixsynOptions structure')
end


%       [W1    -W1*G ]
%  P =  [0      W2   ]
%       [0      W3*G ]
%       [1     -G    ]

P = [1, -G];
if nargin >= 5 && ~isempty(varargin{4})
    W3 = varargin{4};
    P = [0,W3;P];
end
if nargin >= 4 && ~isempty(varargin{3})
    W2 = varargin{3};
    P = [0,W2;P];
end
if nargin >= 3 && ~isempty(varargin{2})
    W1 = varargin{2};
    P = [W1, -W1*G;P];
end



P = mktito(P,1,1);


K = opt.K;
W = sort(opt.W(:));


if isempty(W) && (~isa(G,'frd') || ~isa(G,'idfrd') )
    error('Specify frequency grid in options when using a parametric model');
elseif  isempty(W) && (isa(G,'frd') || isa(G,'idfrd') )
    W = G.Frequency(:);
end

if nargin == 1
    error('Specify initial controller of correct order')
end

if size(K.D,1)~= 1 || size(K.D,2)~= 1
    error('Only SISO supported')
end


gam = sdpvar;
sz = size(P,3);
if strcmpi(opt.norm,'inf') || isempty(opt.norm)
    gamma = gam*ones(length(W),sz);
    obj = gam;
    text_norm = 'inf';
else
    gamma = sdpvar(length(W),sz,'full');
    local_h2_norm = sum(gamma.*kron(([diff(W(:));0] + [W(1);diff(W(:))]),ones(1,sz)));
    obj = max(local_h2_norm)*K.Ts/(2*pi);
    text_norm = 'two';
end

slack = sdpvar;


text_ = [];
if opt.force_integrator || opt.modulus_margin > 0
    if opt.force_integrator
        text_ = 'integrator';
    end
    if opt.modulus_margin > 0
        if isempty(text_)
            text_ = 'appropriate modulus margin';
        else
            text_ = strcat(text_, ' and correct modulus margin');
        end
    end
    fprintf(' -------------------------------------------------------------------\n')
    fprintf(' Searching for controller with %s\n',text_)
    fprintf(' -------------------------------------------------------------------\n')
    K = findGoodController(P,K,W,gamma,slack,opt);
end
if double(slack) > 1e-6
    error(' Could not find appropriate initial controller')
else
    slack = 0;
end

fprintf(' -------------------------------------------------------------------\n')
fprintf('                   Optimizing over H_%s objective\n',text_norm)

fprintf(' -------------------------------------------------------------------\n')
[K,CL,GAM] = solveHinf(P,K,W,gamma,obj,opt);
fprintf(' -------------------------------------------------------------------\n')
varargout{1} = K;
varargout{2} = CL;
varargout{3} = GAM;

end


function [K,CL,GAM] = solveHinf(P,K,W,gamma,obj,opt)
GAM0 = Inf;
for iter = 1 : opt.max_iteration
    
    K = balreal(ss(K));
    [A,B,C,D,Ts] = ssdata(rncf(K));  % compute a state-space realization
    
    Cx = sdpvar(size(C,1),size(C,2),'full');
    assign(Cx,C);
    
    Dx = sdpvar(size(D,1),size(D,2),'full');
    assign(Dx,D);
    
    K_struct = struct('A',A,'B',B,'C',Cx,'D',Dx,'Ts',Ts);
    
    LMIs = gamma>=0;
    if opt.force_integrator
        YX = Cx*inv(eye(size(A))- A)*B+Dx;
        LMIs = [LMIs, YX(1) == 0];
    end
    
    for model = 1 : size(P,3)
        LMIs = [LMIs, getLMIs(P(:,:,model),K_struct,W, gamma(:,model),0,opt.modulus_margin)];
        LMIs = [LMIs, getStabLMIs(P(:,:,model),K_struct,W)];
    end
    JOB = optimize(LMIs,obj,sdpsettings('verbose',0,'cachesolvers',iter));
    
    D1 = double(Dx);
    C1 = double(Cx);
    
    Cb = C1(1,:);
    Ca = C1(2,:);
    
    Db = inv(D1(1,:));
    Da = D1(2,:);
    
    K = ss(A-B*Db*Cb,B*Db, Ca - Da*Db*Cb, Da*Db,Ts);
    
    CL = lft(P,K);
    GAM = sqrt(double(obj));
    
    fprintf(' iteration: %02d | objective %e | %s \n',iter,sqrt(double(obj)), JOB.info);
    if GAM0 - sqrt(double(obj)) > opt.tol
        GAM0 = sqrt(double(obj));
    else
        fprintf(' -------------------------------------------------------------------\n')
        fprintf('                   decrease in objective < tol \n')
        break
    end
    
    if iter == opt.max_iteration
        fprintf(' -------------------------------------------------------------------\n')
        fprintf('                   maximum number of iterations reached \n')
    end
end


end


function K = findGoodController(P,K,W,gamma,slack,opt)

for iter = 1 : opt.max_iteration
    K = balreal(ss(K));
    [A,B,C,D,Ts] = ssdata(rncf(K));  % compute a state-space realization
    
    Cx = sdpvar(size(C,1),size(C,2),'full');
    assign(Cx,C);
    
    Dx = sdpvar(size(D,1),size(D,2),'full');
    assign(Dx,D);
    
    K_struct = struct('A',A,'B',B,'C',Cx,'D',Dx,'Ts',Ts);
    
    LMIs = [gamma>=0,slack>=0];
    if opt.force_integrator
        YX = Cx*inv(eye(size(A))- A)*B+Dx;
        LMIs =  [LMIs,slack>=0,norm(YX(1)) <= slack];
    end
    
    
    for model = 1 : size(P,3)
        lmis = getLMIs(P(:,:,model),K_struct,W, gamma(:,model),slack,opt.modulus_margin);
        if opt.modulus_margin > 0
            LMIs = [LMIs, lmis(2)];
        end
        LMIs = [LMIs, getStabLMIs(P(:,:,model),K_struct,W)];
    end
    
    JOB = optimize(LMIs,slack,sdpsettings('verbose',0,'cachesolvers',iter));
    
    D = double(Dx);
    C = double(Cx);
    
    Cb = C(1,:);
    Ca = C(2,:);
    
    Db = inv(D(1,:));
    Da = D(2,:);
    
    K = ss(A-B*Db*Cb,B*Db, Ca - Da*Db*Cb, Da*Db,Ts);
    % fprintf(' iteration: %02d | objective %e | %s \n',iter,sqrt(double(slack)), JOB.info);
    fprintf(' iteration: %02d | error     %e | %s \n',iter,sqrt(double(slack)), JOB.info);
    if sqrt(double(slack)) < 1e-5
        break
    end
end
end


function LMIs =  getLMIs(P,K_struct,W,gamma,slack,mod_margin)

F = freqresp(P,W);
F11 = F(P.OutputGroup.Y1,P.InputGroup.U1,:);
[a,~,c] = size(F11);
F11 = reshape(F11,a,c);
F12 = F(P.OutputGroup.Y1,P.InputGroup.U2,:);
F12 = reshape(F12,a,c);
F21 = squeeze(F(P.OutputGroup.Y2,P.InputGroup.U1,:));
F22 = squeeze(F(P.OutputGroup.Y2,P.InputGroup.U2,:));

if any(isnan(F22),'all') || any(isnan(F11),'all') || any(isnan(F21),'all') || any(isnan(F12),'all')
    error('Obtained NaN in the frequency response. Check if FRDs are evaluated within the corrects bounds')
end

for ii = 1 : length(W)
    FixedPart(:,ii) = (eye(size(K_struct.A))*exp(1j*W(ii)*K_struct.Ts)- K_struct.A)\K_struct.B ;
end

XY = (K_struct.C*FixedPart).' + ones(length(W),1)*K_struct.D.';
Y = XY(:,1);
X = XY(:,2);



PHI = (Y-F22.*X)./F21;
PHIc= double(PHI);


P = PHI.*conj(PHIc) + PHIc.*conj(PHI) - PHIc.*conj(PHIc);

lambda = gamma;

sz = size(F11,1);
for ii = 1:sz
    
    M(:,ii) = F11(ii,:).'.*PHI+ F12(ii,:).'.*X;
    
end

l = max(min(1e5,1./vecnorm(double(M),2,2)),1e-5).^(2);

LMIs =  rcone_serialized(0.5.*sqrt(l).*lambda,P.*sqrt(l),M.*sqrt(kron(l,ones(1,sz))));
if mod_margin > 0
    LMIs = [LMIs,   rcone_serialized((slack+0.5)*ones(size(W)),P.*l,mod_margin.*Y.*sqrt(l))];
end
end


function rcone = rcone_serialized(x,y,z)
if ~isempty(z)
    % ||z||^2<2xy, x+y>0
    v = 1/sqrt(2);
    T = blkdiag([v,v;v,-v], eye(2*size(z,2)));
    
    Z = T\[x.';y.';real(z).';imag(z).'];
    
    rcone = cone(Z);
else
    rcone = [];
end
end


function LMIs =  getStabLMIs(P,K_struct,W)
F = freqresp(P,W);
G = -squeeze(F(P.OutputGroup.Y2,P.InputGroup.U2,:));

for ii = 1 : length(W)
    FixedPart(:,ii) = (eye(size(K_struct.A))*exp(1j*W(ii)*K_struct.Ts)- K_struct.A)\K_struct.B ;
end

XY = (K_struct.C*FixedPart).' + ones(length(W),1)*K_struct.D.';
Y = XY(:,1);
X = XY(:,2);

P = Y+G.*X;

PExtended = [conj(P(1));P;conj(P(end))];
PcExtended = double(PExtended);

dP = getNormalDirection(PcExtended);


LMIs = real(PExtended(2:end).*conj(dP)) >= 1e-6;
LMIs = [LMIs, real(PExtended(1:end-1).*conj(dP)) >= 1e-6];
end


function n = getNormalDirection(r)
% normal inwards direction for the polygonal chain
for ii = 1 : length(r)-1
    
    A = [real(r(ii)),imag(r(ii))];
    B = [real(r(ii+1)),imag(r(ii+1))];
    M = B - A;
    
    t0 = dot(M, -A) / dot(M, M);
    
    if t0 < 0
        C = A;
    elseif t0 < 1
        C = A + t0 * M;
    else
        C = B;
    end
    n(ii) = C(1) + 1j*C(2);
end
n = n(:)./abs(n(:));
end


