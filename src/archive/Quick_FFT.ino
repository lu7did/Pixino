/*
//example data
int data[256]={10, 42, 47, 19, -20,  -39,  -26,  7,  30, 23, -10,  -42,  -47,  -19,  20,
39, 26, -7, -30,  -23,  10, 42, 47, 19, -19,  -39,  -26,  7,  30, 23, -9, -42,  -47,  -19,
19, 39, 26, -6, -30,  -24,  9,  42, 47, 20, -19,  -39,  -27,  6,  30, 24, -9, -42,  -47,
-20,  19, 39, 27, -6, -30,  -24,  9,  41, 47, 20, -19,  -39,  -27,  6,  30, 24, -9, -41,
-47,  -20,  18, 39, 27, -6, -30,  -24,  8,  41, 47, 20, -18,  -39,  -27,  6,  30, 24, -8,
-41,  -47,  -21,  18, 39, 27, -5, -30,  -24,  8,  41, 47, 21, -18,  -39,  -27,  5,  30, 24,
-8, -41,  -47,  -21,  18, 39, 27, -5, -30,  -24,  8,  41, 47, 21, -18,  -39,  -28,  5,  30,
25, -7, -41,  -47,  -21,  17, 39, 28, -5, -30,  -25,  7,  40, 47, 22, -17,  -39,  -28,  5,
30, 25, -7, -40,  -47,  -22,  17, 39, 28, -4, -30,  -25,  7,  40, 48, 22, -17,  -39,  -28,
4,  29, 25, -7, -40,  -48,  -22,  17, 39, 28, -4, -29,  -25,  7,  40, 48, 22, -17,  -39,  
-28,  4,  29, 25, -6, -40,  -48,  -22,  16, 39, 29, -4, -29,  -25,  6,  40, 48, 23, -16,  
-39,  -29,  4,  29, 25, -6, -40,  -48,  -23,  16, 39, 29, -3, -29,  -26,  6,  39, 48, 23,
-16,  -39,  -29,  3,  29, 26, -6, -39,  -48,  -23,  16, 39, 29, -3, -29,  -26,  5,  39, 48,
23, -15,  -39,  -29,  3,  29, 26, -5, -39,  -48,  -24,  15, 39};
*/

void setup() 
        {
    // Serial.begin(250000);
     
        }

        
void loop() {
/*
float f=Q_FFT(data,256,100);
Serial.println(f);
delay(1000000);
*/
       
            }



//-----------------------------FFT Function----------------------------------------------//
/*
Code to perform High speed (5-7 times faster) and low accuracy FFT on arduino,
This code compromises accuracy for speed,
setup:

1. in[]     : Data array, 
2. N        : Number of sample (recommended sample size 2,4,8,16,32,64,128...)
3. Frequency: sampling frequency required as input (Hz)

It will by default return frequency with max aplitude,

If sample size is not in power of 2 it will be clipped to lower side of number. 
i.e, for 150 number of samples, code will consider first 128 sample, remaining sample  will be omitted.
For Arduino nano, FFT of more than 256 sample not possible due to mamory limitation 
Code by ABHILASH
Contact: abhilashpatel121@gmail.com 
Documentation & deatails: https://www.instructables.com/member/abhilash_patel/instructables/
*/


float Q_FFT(int in[],int N,float Frequency)
{ 

unsigned int Pow2[13]={1,2,4,8,16,32,64,128,256,512,1024,2048}; // declaring this as global array will save 1-2 ms of time


int a,c1,f,o,x;         
byte check=0;
a=N;  
                                 
      for(int i=0;i<12;i++)                 //calculating the levels
         { if(Pow2[i]<=a){o=i;} }
      
int out_r[Pow2[o]]={};   //real part of transform
int out_im[Pow2[o]]={};  //imaginory part of transform
           
x=0;  
      for(int b=0;b<o;b++)                     // bit reversal
         {
          c1=Pow2[b];
          f=Pow2[o]/(c1+c1);
                for(int j=0;j<c1;j++)
                    { 
                     x=x+1;
                     out_im[x]=out_im[j]+f;
                    }
         }

 
      for(int i=0;i<Pow2[o];i++)            // update input array as per bit reverse order
         {
          out_r[i]=in[out_im[i]]; 
          out_im[i]=0;
         }


int i10,i11,n1,tr,ti;
float e;
int c,s;
    for(int i=0;i<o;i++)                                    //fft
    {
     i10=Pow2[i];              // overall values of sine/cosine  
     i11=Pow2[o]/Pow2[i+1];    // loop with similar sine cosine
     e=360/Pow2[i+1];
     e=0-e;
     n1=0;

          for(int j=0;j<i10;j++)
          {
            c=e*j;
  while(c<0){c=c+360;}
  while(c>360){c=c-360;}

          n1=j;
          
          for(int k=0;k<i11;k++)
                 {

       if(c==0) { tr=out_r[i10+n1];
                  ti=out_im[i10+n1];}
  else if(c==90){ tr= -out_im[i10+n1];
                  ti=out_r[i10+n1];}
  else if(c==180){tr=-out_r[i10+n1];
                  ti=-out_im[i10+n1];}
  else if(c==270){tr=out_im[i10+n1];
                  ti=-out_r[i10+n1];}
  else if(c==360){tr=out_r[i10+n1];
                  ti=out_im[i10+n1];}
  else if(c>0  && c<90)   {tr=out_r[i10+n1]-out_im[i10+n1];
                           ti=out_im[i10+n1]+out_r[i10+n1];}
  else if(c>90  && c<180) {tr=-out_r[i10+n1]-out_im[i10+n1];
                           ti=-out_im[i10+n1]+out_r[i10+n1];}
  else if(c>180 && c<270) {tr=-out_r[i10+n1]+out_im[i10+n1];
                           ti=-out_im[i10+n1]-out_r[i10+n1];}
  else if(c>270 && c<360) {tr=out_r[i10+n1]+out_im[i10+n1];
                           ti=out_im[i10+n1]-out_r[i10+n1];}
          
                 out_r[n1+i10]=out_r[n1]-tr;
                 out_r[n1]=out_r[n1]+tr;
                 if(out_r[n1]>15000 || out_r[n1]<-15000){check=1;}
          
                 out_im[n1+i10]=out_im[n1]-ti;
                 out_im[n1]=out_im[n1]+ti;
                 if(out_im[n1]>15000 || out_im[n1]<-15000){check=1;}          
          
                 n1=n1+i10+i10;
                  }       
             }

    if(check==1){                                             // scale the matrics if value higher than 15000 to prevent varible from overloading
                for(int i=0;i<Pow2[o];i++)
                    {
                     out_r[i]=out_r[i]/100;
                     out_im[i]=out_im[i]/100;    
                    }
                     check=0;  
                }           

     }

/*
for(int i=0;i<Pow2[o];i++)
{
Serial.print(out_r[i]);
Serial.print("\t");                                     // un comment to print RAW o/p    
Serial.print(out_im[i]); Serial.println("i");      
}
*/

//---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
int fout,fm,fstp;
float fstep;
fstep=Frequency/N;
fstp=fstep;
fout=0;fm=0;

    for(int i=1;i<Pow2[o-1];i++)               // getting amplitude from compex number
        {
        if((out_r[i]>=0) && (out_im[i]>=0)){out_r[i]=out_r[i]+out_im[i];}
   else if((out_r[i]<=0) && (out_im[i]<=0)){out_r[i]=-out_r[i]-out_im[i];}
   else if((out_r[i]>=0) && (out_im[i]<=0)){out_r[i]=out_r[i]-out_im[i];}
   else if((out_r[i]<=0) && (out_im[i]>=0)){out_r[i]=-out_r[i]+out_im[i];}
   // to find peak sum of mod of real and imaginery part are considered to increase speed
        
out_im[i]=out_im[i-1]+fstp;
if (fout<out_r[i]){fm=i; fout=out_r[i];}
         /*
         Serial.print(out_im[i]);Serial.print("Hz");
         Serial.print("\t");                            // un comment to print freuency bin    
         Serial.println(out_r[i]); 
          */
        }


float fa,fb,fc;
fa=out_r[fm-1];
fb=out_r[fm]; 
fc=out_r[fm+1];
fstep=(fa*(fm-1)+fb*fm+fc*(fm+1))/(fa+fb+fc);

return(fstep*Frequency/N);
}
    

//------------------------------------------------------------------------------------//
