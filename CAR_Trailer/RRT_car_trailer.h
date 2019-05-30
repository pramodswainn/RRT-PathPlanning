#include "build_car_trailer.h"

int A1 = carL1*carL1;
int A2 = carL2*carL2;
double dt=3;
random_source R;
window W(625,625);


//=================================================================

double dist(car_q p, car_q q)
{
double d;
double dcita1, dcita2;

dcita1=p.cita1-q.cita1;
if (dcita1>PI)
	dcita1-=2*PI;
else if (dcita1<-PI)
	dcita1+=2*PI;

dcita2=p.cita2-q.cita2;
if (dcita2>PI)
	dcita2-=2*PI;
else if (dcita2<-PI)
	dcita2+=2*PI;

d = (p.x-q.x)*(p.x-q.x)+ (p.y-q.y)*(p.y-q.y)+ A1*dcita1*dcita1+ A2*dcita2*dcita2;

d = pow(d,0.5);

return(d);
}

node Nearest_Vertex(car_q q, GRAPH<car_q, car_u> &G)
{

car_q   vr;
node  vn_node, vr_node;
double d=10000,dr;

forall_nodes(vr_node,G){
	vr = G.inf(vr_node);
	dr=dist(vr, q);
	if (dr < d){
		vn_node=vr_node;
		d=dr;
		}
	}
return(vn_node);

}

car_q Rand_Conf()
{
car_q q;
q.x=-1;
q.y=-1;
q.cita1=-5;
q.cita2=-5;

while(q.x<0 || q.x>100){
	R >> q.x;
	q.x*=100;
	}

while(q.y<0 || q.y>100){
	R >> q.y;
	q.y*=100;
	}

while(q.cita1<=-PI || q.cita1>PI){
	R >> q.cita1;
	q.cita1=q.cita1*2*PI-PI;
	}

while(q.cita2<=-PI/2 || q.cita2>PI/2){
	R >> q.cita2;
	q.cita2=q.cita2*PI-PI/2;
	}
return(q);

}

car_q Goal_Region_Biased_Conf(car_q goal, double PG, double d)
{
double x;
R >> x;
car_q q;

if (x< PG)
	return (goal);

else if (x< 0.5){
	q=Rand_Conf();
	while (dist(q,goal)>2*d)
		q=Rand_Conf();
	return(q);
	}
else return(Rand_Conf());

}

car_q New_conf(car_q vn,car_u u)
{
car_q new_q;

new_q.x = vn.x+u.v*dt*cos(vn.cita1)*cos(u.fi);
new_q.y = vn.y+u.v*dt*sin(vn.cita1)*cos(u.fi);
new_q.cita1 = vn.cita1+(u.v*dt/carL1)*sin(u.fi);
new_q.cita2 = vn.cita2+(u.v*dt/carL2)*sin(vn.cita1-vn.cita2)*cos(u.fi);

return(new_q);
}

car_u Optm_Input(car_q vn, car_q q, list<polygon> obsts)
{
int d1, d2, d=10000, dr;
int k1=0, k2=0;
car_q new_q, p1, p2, p;
car_u optm_u, u1, u2, u;
polygon carBody, carTr, obs;
list<polygon> intersect1, intersect2;
list_item it;

for (int j=0;j<2;j++){
   if (j==0){
	u.v=1;
	u.fi=-MAXfi;
	}

   else {
	u.v=-1;
	u.fi=-MAXfi;
	}

   for (int i=0;i<21;i++)
	{

	p=New_conf(vn,u);
	dr=dist(p, q);
	if(dr<d){
		d=dr;
		new_q=p;
		optm_u=u;
		}		
	u.fi+=(2.0*MAXfi)/20.0;
	}
   }
//cout << endl << "-----" << endl << optm_u;

u1=optm_u;
u2=optm_u;

carBody=drawCarBody(vector(new_q.x,new_q.y), new_q.cita1);
carTr=drawTrailer(vector(new_q.x,new_q.y), new_q.cita1+new_q.cita2);


if (outBox(carBody)==1 || outBox(carTr)){
	u1.fi=u1.fi+PI/20;
	u2.fi=u2.fi-PI/20;
	p1=New_conf(vn,u1);
	p2=New_conf(vn,u2);
	k1=1;
	k2=1;
	}

else	{
	it=obsts.first();
	while(it!=nil){
		obs=obsts.inf(it);
		intersect1=obs.intersection(carBody);
		intersect2=obs.intersection(carTr);
		if (!intersect1.empty() || !intersect2.empty()){
			u1.fi=u1.fi+PI/20;
			u2.fi=u2.fi-PI/20;
			p1=New_conf(vn,u1);
			p2=New_conf(vn,u2);
			k1=1;
			k2=1;
			break;
			}
		it=obsts.succ(it);
		}
	}

while(k1==1){
	k1=0;
	d1=dist(p1, q);
	carBody=drawCarBody(vector(p1.x,p1.y), p1.cita1);
	carTr=drawTrailer(vector(p1.x,p1.y), p1.cita1+p1.cita2);

	if (outBox(carBody)==1 || outBox(carTr)){
		u1.fi=u1.fi+PI/20;
		p1=New_conf(vn,u1);
		k1=1;
		}
	else	{
		it=obsts.first();
		while(it!=nil){
			obs=obsts.inf(it);
			intersect1=obs.intersection(carBody);
			intersect2=obs.intersection(carTr);
			if(!intersect1.empty() || !intersect2.empty()){ 
				u1.fi=u1.fi+PI/20;
				p1=New_conf(vn,u1);
				k1=1;
				break;
				}
			it=obsts.succ(it);
			}
		}
	if(abs(u1.fi)>MAXfi){
		d1=10000;
		break;
		}
	}

while(k2==1){
	k2=0;
	d2=dist(p2, q);
	carBody=drawCarBody(vector(p2.x,p2.y), p2.cita1);
	carTr=drawTrailer(vector(p2.x,p2.y), p2.cita1+p2.cita2);

	if (outBox(carBody)==1 || outBox(carTr)){
		u2.fi=u2.fi-PI/20;
		p2=New_conf(vn,u2);
		k2=1;
		}
	else	{
		it=obsts.first();
		while(it!=nil){
			obs=obsts.inf(it);
			intersect1=obs.intersection(carBody);
			intersect2=obs.intersection(carTr);
			if(!intersect1.empty() || !intersect2.empty()){ 
				u2.fi=u2.fi-PI/20;
				p2=New_conf(vn,u2);
				k2=1;
				break;
				}
			it=obsts.succ(it);
			}
		}

	if(abs(u2.fi)>MAXfi){
		d2=10000;
		break;
		}
	}

if (d1==10000 && d2==10000)
	optm_u.v=0;		
else if (d1<=d2)
	optm_u=u1;
else optm_u=u2;

//cout << endl << optm_u;
return(optm_u);

}



GRAPH<car_q, car_u> Build_RRT(car_q q_init, car_q goal ,list<polygon> obsts)
{
GRAPH<car_q, car_u> G;
car_q q, vn;
node vn_node,qn_node;
car_q new_q;
car_u new_u;
double d=10000;
double PG=0.05;
char c;

G.new_node(q_init);

while (d>1) {
    for(int i=0;i<1000; i++){
	q=Goal_Region_Biased_Conf(goal, PG, d);
	vn_node=Nearest_Vertex(q,G);
	vn=G.inf(vn_node);

	new_u=Optm_Input(vn,q,obsts);
	new_q=New_conf(vn,new_u);

	qn_node = G.new_node(new_q);
	G.new_edge(vn_node,qn_node,new_u);
	
	W.draw_segment(point(vn.x, vn.y), point(new_q.x,new_q.y), col_green);	
	}

	vn_node=Nearest_Vertex(goal,G);
	vn = G.inf(vn_node);
	d = dist(vn, goal);
	cout << endl << "d= " << d << endl << "Continue? ";
	cin >> c;
	if(c!='y') break;
	}

return(G);
}


list<car_move> Car_Path(car_q q_init, car_q goal, GRAPH<car_q, car_u> &G, list<polygon> obsts)
{
node vn_node, root_node;
edge e;
car_q vn;
list<car_move> path;
car_move carM;
polygon obs;

vn_node=Nearest_Vertex(goal,G);

root_node = G.first_node();

while (vn_node !=  root_node){
	e=G.first_in_edge(vn_node);
	carM.q=G.inf(vn_node);
	carM.u=G.inf(e);
	path.push(carM);
	vn_node=G.source(e);
	vn = G.inf(vn_node);
	W.set_line_width(3);
	W.draw_segment(point(vn.x, vn.y), point((carM.q).x,(carM.q).y), col_red);	
	}
W.set_line_width(1);

carM.q=G.inf(vn_node);
(carM.u).v=0;
(carM.u).fi=0;
path.push(carM);

return(path);
}

