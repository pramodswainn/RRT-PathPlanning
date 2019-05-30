
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Things that are used from the LEDA library
#include <LEDA/graph.h>
#include <LEDA/list.h>
#include <LEDA/point.h>
#include <LEDA/polygon.h>
#include <LEDA/ps_file.h>
#include <LEDA/random_source.h>
#include <LEDA/window.h>

#define max_num_Vertices 100
#define max_num_Polygons 100
#define PI 3.1415926

extern window W;
extern window W2;
extern color col_white, col_gb, col_black, col_grey, col_red, col_green;


int cmp(point const& p1, point const&  p2)
{
if (p1.xcoord()<p2.xcoord())
  return(-1);
else if (p1.xcoord()>p2.xcoord())
  return(1);
else if (p1.ycoord()<p2.ycoord())
  return(-1);
else return(1);
}

list<polygon> obst(void)
{
list<polygon> plys;
int i,k,n=100;

int st=0;

array<point> temp(3);
array<point> p(max_num_Vertices);
list<point> L_upper,L_lower;

//ps_file F(15.0,15.0,"hw1.ps");

//*************** Obstacles(Polygons) **************************
while(st==0)
	{
	i=-1;
	//-------draw a convex-----------
	while(1)
  		{
		i++;
    		k=W2.get_mouse(p[i]);
    		if (k==MOUSE_BUTTON(2)){n=i;break;}
    		else if (k==MOUSE_BUTTON(1))
      			{
			W2.draw_point(p[i]);
      			}
    		else if (k==MOUSE_BUTTON(3)){n=i;st=1;break;}
    		else i--;
  		}
      

	p.sort(cmp,0,n-1);

	L_upper.clear();
	L_upper.clear();

	L_upper.push(p[0]);
	L_upper.push(p[1]);

	for (i=2;i<n;i++){
  		L_upper.push(p[i]);

  		while(L_upper.length()>2){
    			temp[0]=L_upper.contents(L_upper[0]);
    			temp[1]=L_upper.contents(L_upper[1]);
    			temp[2]=L_upper.contents(L_upper[2]);

    			if (!right_turn(temp[2],temp[1],temp[0])){
      				L_upper.erase(L_upper[1]);
//      				m=L_upper.length(); 
//      				cout << m;
    				}
    			else break;
    			}
		}

	L_upper.reverse_items();
      
	L_lower.push(p[n-1]);
	L_lower.push(p[n-2]);

	for (i=n-3;i>=0;i--){
  		L_lower.push(p[i]);
  		while(L_lower.length()>2){
    			temp[0]=L_lower.contents(L_lower[0]);
    			temp[1]=L_lower.contents(L_lower[1]);
    			temp[2]=L_lower.contents(L_lower[2]);

    			if (!right_turn(temp[2],temp[1],temp[0])){
      				L_lower.erase(L_lower[1]);
    				}
    			else break;
  			}
		}

	L_lower.pop();
	L_lower.Pop();    

	L_lower.reverse_items();
      	L_upper.conc(L_lower);

//	L_upper.print();

	plys.push(polygon(L_upper));
	W.draw_filled_polygon(polygon(L_upper), col_grey);
	W2.draw_filled_polygon(polygon(L_upper), col_grey);

	//F.draw_polygon(L_upper);
	//a convex done
	
	}//============while(st==0)===============

return(plys);

}








