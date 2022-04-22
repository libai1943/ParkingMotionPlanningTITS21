param BasicParameters{i in {1..20}};
param SixBoundaryValues{i in {1..6}};
param Nfe == BasicParameters[16];
param vmax == BasicParameters[12];
param amax == BasicParameters[13];
param phymax == BasicParameters[14];
param wmax == BasicParameters[15];
param L_wheelbase == BasicParameters[5];
param Lrc == BasicParameters[10];
param Lfc == BasicParameters[11];
param w_penalty == BasicParameters[17];
param w_a == BasicParameters[18];
param w_w == BasicParameters[19];
param w_phy == BasicParameters[20];
param r == BasicParameters[9];
param xmin == BasicParameters[1] + r;
param xmax == BasicParameters[2] - r;
param ymin == BasicParameters[1] + r;
param ymax == BasicParameters[2] - r;

var tf >= 0.1;
var hi = tf / (Nfe - 1);
param STC_rear{i in {1..Nfe}, j in {1..4}};
param STC_front{i in {1..Nfe}, j in {1..4}};

var x{i in {1..Nfe}};
var y{i in {1..Nfe}};
var theta{i in {1..Nfe}};
var v{i in {1..Nfe}};
var a{i in {1..Nfe}};
var phy{i in {1..Nfe}};
var w{i in {1..Nfe}};
var xf{i in {1..Nfe}};
var yf{i in {1..Nfe}};
var xr{i in {1..Nfe}};
var yr{i in {1..Nfe}};

minimize obj: 
tf + w_a * sum{i in {1..Nfe}}(a[i]^2)+ w_phy * sum{i in {1..Nfe}}(phy[i]^2)+ w_w * sum{i in {1..Nfe}}(w[i]^2) +
w_penalty * (sum{i in {2..Nfe}}((x[i] - x[i-1] - hi * v[i] * cos(theta[i]))^2 + (y[i] - y[i-1] - hi * v[i] * sin(theta[i]))^2 + (v[i] - v[i-1] - hi * a[i])^2 + (theta[i] - theta[i-1] - hi * tan(phy[i]) * v[i] / L_wheelbase)^2 + (phy[i] - phy[i-1] - hi * w[i])^2 + (xf[i] - x[i] - Lfc * cos(theta[i]))^2 + (yf[i] - y[i] - Lfc * sin(theta[i]))^2 + (xr[i] - x[i] - Lrc * cos(theta[i]))^2 + (yr[i] - y[i] - Lrc * sin(theta[i]))^2) + (sin(theta[Nfe]) - sin(SixBoundaryValues[6]))^2 + (cos(theta[Nfe]) - cos(SixBoundaryValues[6]))^2);

s.t. time_limit:
tf <= Nfe * 0.5;

s.t. EQ_init_x :
x[1] = SixBoundaryValues[1];
s.t. EQ_init_y :
y[1] = SixBoundaryValues[2];
s.t. EQ_init_theta :
theta[1] = SixBoundaryValues[3];
s.t. EQ_init_v :
v[1] = 0;
s.t. EQ_init_a :
a[1] = 0;
s.t. EQ_init_phy :
phy[1] = 0;
s.t. EQ_init_w :
w[1] = 0;

s.t. EQ_end_x :
x[Nfe] = SixBoundaryValues[4];
s.t. EQ_end_y :
y[Nfe] = SixBoundaryValues[5];
s.t. EQ_end_v :
v[Nfe] = 0;
s.t. EQ_end_a :
a[Nfe] = 0;
s.t. EQ_end_phy :
phy[Nfe] = 0;
s.t. EQ_end_w :
w[Nfe] = 0;

s.t. Bonds_phy {i in {1..Nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_v {i in {1..Nfe}}:
-vmax <= v[i] <= vmax;
s.t. Bonds_w {i in {1..Nfe}}:
-wmax <= w[i] <= wmax;
s.t. Bonds_a {i in {1..Nfe}}:
-amax <= a[i] <= amax;

s.t. Box_on_xf {i in {1..Nfe}}:
STC_front[i,1] <= xf[i] <= STC_front[i,2];
s.t. Box_on_yf {i in {1..Nfe}}:
STC_front[i,3] <= yf[i] <= STC_front[i,4];
s.t. Box_on_xr {i in {1..Nfe}}:
STC_rear[i,1] <= xr[i] <= STC_rear[i,2];
s.t. Box_on_yr {i in {1..Nfe}}:
STC_rear[i,3] <= yr[i] <= STC_rear[i,4];

data;
param: BasicParameters := include BasicParameters;
param: STC_rear := include STC_rear;
param: STC_front := include STC_fron;
param: SixBoundaryValues := include SixBoundaryValues;