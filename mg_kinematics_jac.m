NewtonianFrame N

RigidBody A, B, C
Point F(C)

Constant W, L
Constant LA, LX, LZ, L3

Constant Bcmx, Bcmz % both in +B> direction
Constant Ccmz % in +Cz> direction

Variables qA'', qB'', qC''
SetGeneralizedSpeeds(qA', qB', qC')

A.SetMassInertia(mA, IAxx, IAyy, IAzz)
B.SetMassInertia(mB, IBxx, IByy, IBzz)
C.SetMassInertia(mC, ICxx, ICyy, ICzz)

A.RotatePositiveY(N, qA)
B.RotatePositiveX(A, qB)
C.RotatePositiveY(B, qC)

Ao.Translate(No, -W/2 * Ny> -L/2 * Nx>)
Bo.Translate(Ao, LA * Ay> )
Co.Translate(Bo, -LX * Bx> - LZ * Bz>)
F.Translate(Co, -L3 * Cz>)

Acm.Translate(Ao, 0>)
Bcm.Translate(Bo, Bcmx * Bx> + Bcmz * Bz>)
Ccm.Translate(Co, Ccmz * Cz>)

No_R_F> = F.GetPosition(No)
Fx = Dot(No_R_F>, Nx>)
Fy = Dot(No_R_F>, Ny>)
Fz = Dot(No_R_F>, Nz>)

N_v_F> = F.GetVelocity(N)
vx = Dot(N_v_F>, Nx>)
vy = Dot(N_v_F>, Ny>)
vz = Dot(N_v_F>, Nz>)

% Dynamics
Dynamics = System.GetKaneDynamics()

% Jacobian
D([Fx;Fy;Fz],[qA,qB,qC])

% FK
Fx
Fy
Fz

