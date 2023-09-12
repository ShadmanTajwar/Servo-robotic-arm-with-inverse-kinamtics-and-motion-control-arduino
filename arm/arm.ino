double sgn(double x);
#include <Servo.h>

float y[4]= {0, 0, 0, 0};
float tdjk= 200;  //time interval microseconds
float acc= 0.002; //acceleration,

double ti[4]= {0, 0, 0, 0};
double tjkk[4]= {0, 0, 0, 0};
double tdijo[4]= {tdjk, tdjk, tdjk, tdjk};
double tdijp[4]= {tdjk, tdjk, tdjk, tdjk};
float tk[4]= {0, 0, 0, 0};
float tj[4]= {0, 0, 0, 0};
float tjk[4]= {0, 0, 0, 0};
float mi[4]= {0, 0, 0, 0};
float mf[4]= {0, 0, 0, 0};
float ak[4]= {0, 0, 0, 0}; 
float a[4]= {0, 0, 0, 0};
float b[4]= {0, 0, 0, 0};
float c[4]= {0, 0, 0, 0};
int m[4]= {0, 0, 0, 0}; //blend
int l[4]= {0, 0, 0, 0}; //linear
int o[4]= {0, 0, 0, 0};
int p[4]= {0, 0, 0, 0};
unsigned long t[4]= {0, 0, 0, 0};
unsigned long td[4]= {0, 0, 0, 0};
int n=0;
float qu[4]= {0, 0, 0, 0};
float qv[4]= {0, 0, 0, 0};
float qw[4]= {0, 0, 0, 0};
float qc[4]= {0, 0, 0, 0};
float qum= 0;
float qvm= 0;
float qwm= 0;
float qcm= 0;
int uc= 0;
int vc= 0;
int wc= 0;
int cc= 0;

int z0= 0; //z offset for target

Servo j1,j2,j3,j4,j5,j6,g6;
int d1=90, d4=121, d6=0, l0=28, l2=150; //dh parameters
float q1, q2, q3, q6; 
float u(float t06[12][4][4]); 
float v(float t06[12][4][4]);
float w(float t06[12][4][4]);
float g(float t06[12][4][4]);

float t07[][4][4]={{{1,0,0,0},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,0},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}}, 
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}}, 
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}}, 
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}}, 
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}}, 
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,45-z0},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,-120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,70-z0},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,120},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,0},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,0},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,0},{0,0,-1,170},{0,1,0,240},{0,0,0,1}},
                      {{1,0,0,0},{0,0,-1,170},{0,1,0,240},{0,0,0,1}}};
                      
void setup() {
  Serial.begin(9600);
  j1.attach(2);
  j2.attach(3);  
  j3.attach(4);
  j6.attach(5);
  g6.attach(6);
  g6.write(0);
  delay(3000);
}

void loop() 
{
  parabolicBlend(0, 5);
  g6.write(120);
  delay(1000);
  parabolicBlend(4, 13);// next of last points
  g6.write(0);
  delay(1000);
  parabolicBlend(12, 21);
  g6.write(120);
  delay(1000);
  parabolicBlend(20, 29);
  g6.write(0);
  delay(1000);
  parabolicBlend(28, 33);
  g6.write(120);
  delay(1000);
  parabolicBlend(32, 41);
  g6.write(0);
  delay(1000);
  parabolicBlend(40, 49);
  g6.write(120);
  delay(1000);
  parabolicBlend(48, 57);
  g6.write(0);
  delay(1000);
  parabolicBlend(56, 62);
  g6.write(0);
  delay(1000);
}

void parabolicBlend( int q1, int q2)
{
  for(n= 0; n< 4; n++)
  {
    ti[n]= millis();
    m[n]= q1;
    l[n]= q1; 
  }

  qu[0]= u(t07, l[n]);
  qu[1]= u(t07, (l[n]+1));
  qu[2]= u(t07, (l[n]+2));

  qv[0]= v(t07, l[n]);
  qv[1]= v(t07, (l[n]+1));
  qv[2]= v(t07, (l[n]+2));

  qw[0]= w(t07, l[n]);
  qw[1]= w(t07, (l[n]+1));
  qw[2]= w(t07, (l[n]+2));

  qc[0]= g(t07, l[n]);
  qc[1]= g(t07, (l[n]+1));
  qc[2]= g(t07, (l[n]+2));

  uc= l[n];
  vc= l[n];
  wc= l[n];
  cc= l[n];

  while(l[3]<= q2)
  {
    for(n= 0; n< 4; n++)
    {
      t[n]= millis();
      td[n]= t[n]- ti[n]; 
      
      switch(n)
      {
        case 0:
           if(uc!= l[n]) //if change in l[n] then find new qv values
          { 
            qu[0]= qu[1];
            qu[1]= qu[2];
            qu[2]= u(t07, (l[n]+2));
            uc++;
          }
          mi[n]= (qu[1]-qu[0])/(float)tdjk; //final-intial, slope of initial, inverse kinematics here
          mf[n]= (qu[2]-qu[1])/(float)tdjk; //final slope, inverse kinematics her
          break;

        case 1:
          if(vc!= l[n]) //if change in l[n] then find new qv values
          { 
            qv[0]= qv[1];
            qv[1]= qv[2];
            qv[2]= v(t07, (l[n]+2));
            vc++;
          }
          mi[n]= (qv[1]-qv[0])/(float)tdjk; //final-intial, slope of initial, inverse kinematics here
          mf[n]= (qv[2]-qv[1])/(float)tdjk; //final slope, inverse kinematics her
          break;

        case 2:
          if(wc!= l[n]) //if change in l[n] then find new qv values
          { 
            qw[0]= qw[1];
            qw[1]= qw[2];
            qw[2]= w(t07, (l[n]+2));
            wc++;
          }
          mi[n]= (qw[1]-qw[0])/(float)tdjk; //final-intial, slope of initial, inverse kinematics here
          mf[n]= (qw[2]-qw[1])/(float)tdjk; //final slope, inverse kinematics her
          break;

        case 3:
          if(cc!= l[n]) //if change in l[n] then find new qv values
          { 
            qc[0]= qc[1];
            qc[1]= qc[2];
            qc[2]= g(t07, (l[n]+2));
            cc++;
          }
          mi[n]= (qc[1]-qc[0])/(float)tdjk; //final-intial, slope of initial, inverse kinematics here
          mf[n]= (qc[2]-qc[1])/(float)tdjk; //final slope, inverse kinematics her
          break;

        default:
          break;
      }

      if(mi[n]== mf[n])
      {
        if(t[n]> (tjkk[n]+ti[n]) && t[n]<= (tdijo[n]+ti[n]))
        {
          switch(n)
          {
            case 0:
              y[n]= qu[0];
              break;

            case 1:
              y[n]= qv[0];
              break;

            case 2:
              y[n]= qw[0];
              break;

             case 3:
              y[n]= qc[0];
              break;

            default:
              break;
          }
        }

        if(t[n]> (tdijo[n]+ti[n]))
        { 
          tjkk[n]= tdijo[n];
          tdijo[n]+= tdjk; //tjkk[n]= tdjk*(o[n]+1);
          l[n]++;
          tj[n]= 0;
        }
      }

      else
      {      
        ak[n]= sgn(mf[n]-mi[n])*acc; //acceleration
        tk[n]= (mf[n]-mi[n])/ak[n]; //if mf[n]-mi[n]=0 then see tk=0
        tjk[n]= tdjk-(tk[n]/2)-(tj[n]/2);

        if(t[n]> (tjk[n]+tjkk[n]+ti[n]) && t[n]< (abs(tk[n])+tjk[n]+tjkk[n]+ti[n])) //blend part
        {
          a[n]= ak[n]/2;
          b[n]= mi[n];
          
          switch(n)
          {
            case 0:
              c[n]= qu[1]-(mi[n]*tk[n]/2);
              break;

            case 1:
              c[n]= qv[1]-(mi[n]*tk[n]/2);
              break;
            
            case 2:
              c[n]= qw[1]-(mi[n]*tk[n]/2);
              break;

            case 3:
              c[n]= qc[1]-(mi[n]*tk[n]/2);
              break;

            default:
              break;
          }
          y[n]= (a[n]*(float(td[n])-tjkk[n]-tjk[n])*(float(td[n])-tjkk[n]-tjk[n]))+(b[n]*(float(td[n])-tjkk[n]-tjk[n]))+c[n]; //y[n]= (a[n]*pow((t[n]-tjkk[n]-tjk[n]),2))+(b[n]*(t[n]-tjkk[n]-tjk[n]))+c[n];
        }
          
        if(t[n]>= (abs(tk[n])+tjk[n]+tjkk[n]+ti[n])) //(t[n]> (abs(tk[n])+tjk[n]+tjkk[n]))
        {
          tjkk[n]= tjk[n]+tk[n]+tjkk[n];
          tj[n]= tk[n];
          tdijo[n]+= tdjk;
          l[n]++;
        }
          
        if(t[n]<= (tjk[n]+tjkk[n])+ti[n]) //linear part
        {
          switch(n)
          {
            case 0:
              qum= u(t07, m[n]);
              mi[n]= (u(t07, (m[n]+1))-qum)/(float)tdjk;
              y[n]= (mi[n]*(float(td[n])-(tdjk*p[n])))+qum;
              break;

            case 1:
              qvm= v(t07, m[n]);
              mi[n]= (v(t07, (m[n]+1))-qvm)/(float)tdjk;
              y[n]= (mi[n]*(float(td[n])-(tdjk*p[n])))+qvm;
              break;

            case 2:
              qwm= w(t07, m[n]);
              mi[n]= (w(t07, (m[n]+1))-qwm)/(float)tdjk;
              y[n]= (mi[n]*(float(td[n])-(tdjk*p[n])))+qwm;
              break;

            case 3:
              qcm= g(t07, m[n]);
              mi[n]= (g(t07, (m[n]+1))-qcm)/(float)tdjk;
              y[n]= (mi[n]*(float(td[n])-(tdjk*p[n])))+qcm;
              break;

            default:
              break;
          }
        }

        if(t[n]> tdijp[n]+ti[n])
        {
          tdijp[n]+= tdjk;
          p[n]++;
          m[n]++;
        }
      }
    }

    j1.write(y[0]);
    j2.write(y[1]); 
    j3.write(y[2]);
    j6.write(y[3]+20);

    /*Serial.print(y[0]);
    Serial.print(" ");
    Serial.print(y[1]);
    Serial.print(" ");
    Serial.print(y[2]);
    Serial.print(" ");
    Serial.print(y[3]);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.print(180);
    Serial.print(" ");
    Serial.println(" ");*/
  
  }

  for(n= 0; n< 4; n++)
  {
    tjkk[n]= 0;
    tdijo[n]= tdjk;
    tdijp[n]= tdjk; 
    tj[n]= 0;
    o[n]= 0;
    p[n]= 0;
  }
}

float u(float t06[][4][4], int n)
{
    float xc=t06[n][0][3]-(d6*t06[n][0][2]);
    float yc=t06[n][1][3]-(d6*t06[n][1][2]);
    float zc=t06[n][2][3]-(d6*t06[n][2][2]);

    float q1=atan(yc/xc);
    
    if(xc>1 && yc>1)
    {
        q1=(q1*180/M_PI);
    }
    if(xc<1 && yc>1)
    {
        q1=(q1*180/M_PI)+180;
    }
    if(xc==0 && yc>1)
    {
        q1=90;
    }
    
    return(q1); 
}

float v(float t06[][4][4], int n)
{
    float xc=t06[n][0][3]-(d6*t06[n][0][2]);
    float yc=t06[n][1][3]-(d6*t06[n][1][2]);
    float zc=t06[n][2][3]-(d6*t06[n][2][2]);

    float r=sqrt((xc*xc)+(yc*yc))-l0; //r=sqrt((pow(xc,2))+pow(yc,2))-l0;
    float pwz=zc-d1;
    float s=sqrt((r*r)+(pwz*pwz)); //s=sqrt((pow(r,2))+pow(pwz,2));
    float a=atan(pwz/r);
    float b=fabs(acos(((s*s)+(l2*l2)-(d4*d4))/(2*l2*s))); //b=fabs(acos((pow(s,2)+pow(l2,2)-pow(d4,2))/(2*l2*s)));
    float q2=1.5708-a-b; //q2=(M_PI/2)-a-b;
    q2=(q2*57.2958)+90; //q2=(q2*180/M_PI)+90;
    
    return(q2); 
}

float w(float t06[][4][4], int n)
{
    float xc=t06[n][0][3]-(d6*t06[n][0][2]);
    float yc=t06[n][1][3]-(d6*t06[n][1][2]);
    float zc=t06[n][2][3]-(d6*t06[n][2][2]);

    float r=sqrt((xc*xc)+(yc*yc))-l0; //r=sqrt((pow(xc,2))+pow(yc,2))-l0;
    float pwz=zc-d1;
    float s=sqrt((r*r)+(pwz*pwz)); //s=sqrt((pow(r,2))+pow(pwz,2));
    float g=fabs(acos(((l2*l2*1.0)+(d4*d4)-(s*s))/(2*l2*(float)(d4)))); //2*l2*d4=36000, g=fabs(acos((pow(l2,2)+pow(d4,2)-pow(s,2))/(36000))) !
    float q3=(1.5708)-g; //q3=(M_PI/2)-g;
    q3=((-1)*q3*57.2958)+90;
    
    return(q3); 
}

float g(float t06[][4][4], int n)
{
    float xc=t06[n][0][3]-(d6*t06[n][0][2]);
    float yc=t06[n][1][3]-(d6*t06[n][1][2]);
    float zc=t06[n][2][3]-(d6*t06[n][2][2]);

    float r=sqrt((xc*xc)+(yc*yc))-l0; //r=sqrt((pow(xc,2))+pow(yc,2))-l0;
    float pwz=zc-d1;
    float s=sqrt((r*r)+(pwz*pwz)); //s=sqrt((pow(r,2))+pow(pwz,2));
    float a=atan(pwz/r);
    float b=fabs(acos(((s*s)+(l2*l2)-(d4*d4))/(2*l2*s))); //b=fabs(acos((pow(s,2)+pow(l2,2)-pow(d4,2))/(2*l2*s)));
    float g=fabs(acos(((l2*l2*1.0)+(d4*d4)-(s*s))/(2*l2*(float)(d4)))); //g=fabs(acos((pow(l2,2)+pow(d4,2)-pow(s,2))/(36300)));//2*l2*d4=36000 
    float p=180-(57.2957*(b+g)); //p=180-(180*(b+g)/M_PI);
    q6=p-(a*57.2957)+90; //q4=p-(a*180/M_PI);
    
    return(q6); 
}

double sgn(double x)
{
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  if (x == 0.0) return 0;
  return x;
}
