/*
 * Copyright (C) 1998, 2000-2007, 2010, 2011, 2012, 2013 SINTEF ICT,
 * Applied Mathematics, Norway.
 *
 * Contact information: E-mail: tor.dokken@sintef.no                      
 * SINTEF ICT, Department of Applied Mathematics,                         
 * P.O. Box 124 Blindern,                                                 
 * 0314 Oslo, Norway.                                                     
 *
 * This file is part of SISL.
 *
 * SISL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version. 
 *
 * SISL is distributed in the hope that it will be useful,        
 * but WITHOUT ANY WARRANTY; without even the implied warranty of         
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with SISL. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * In accordance with Section 7(b) of the GNU Affero General Public
 * License, a covered work must retain the producer line in every data
 * file that is created or manipulated using SISL.
 *
 * Other Usage
 * You can be released from the requirements of the license by purchasing
 * a commercial license. Buying such a license is mandatory as soon as you
 * develop commercial activities involving the SISL library without
 * disclosing the source code of your own applications.
 *
 * This file may be used in accordance with the terms contained in a
 * written agreement between you and SINTEF ICT. 
 */

#include "sisl-copyright.h"

/*
 *
 * $Id: sh1928.c,v 1.2 2001-03-19 15:59:07 afr Exp $
 *
 */

#define SH1928

#include "sislP.h"

#if defined(SISLNEEDPROTOTYPES)
void
      sh1928(double etau[],int ik,int in,int idim,double et[],
	     double ed[],int im,int ilend,int irend,
	     double ea[],int inh,int nfirst[],int nlast[],
	     double eb[],double ec[],int n2sta[],int *jstat)
#else
void sh1928(etau,ik,in,idim,et,ed,im,ilend,irend,ea,inh,nfirst,nlast,
	    eb,ec,n2sta,jstat)
   double etau[];
   int ik;
   int in;
   int idim;
   double et[];
   double ed[];
   int im;
   int ilend;
   int irend;
   double ea[];
   int inh;
   int nfirst[];
   int nlast[];
   double eb[];
   double ec[];
   int n2sta[];
   int *jstat;
#endif     
/*
*********************************************************************
* 
* PURPOSE    : To set up the normal equations for solving the linear
*              system ea*x=ed in a wighted least squares sense. The
*              least squares problem stems from the problem of approximating
*              the spline with knot vector et and B-spline coefficients 
*              ed from the subspace generated by the knot vector etau
*              which is a subsequence of et. The aim is to minimize a
*              weighted 2-norm of the coefficients of the difference between
*              a spline from the spline space generated by etau and the
*              given spline in the spline space gernerated by et expressed
*              in the latter space.
*              However, there are side constraints which means that the first
*              ilend and the last irend unknowns have already been computed
*              and the right-hand-side adjusted correspondingly so columns
*              0,...,ilend-1 and in-irend,...,in-1 of ea are to be ignored.
* 
* INPUT      : etau   - Real array of length (in+ik) containing the  
*                       knot vector of the approximatin space.
*	       ik     - The order of the spline space.
*              in     - The dimension of the spline space corresponding
*                       to etau.
*              idim   - The dimension of the geometry space.
*              et     - Real array of length (im+ik) containing the refined
*                       knot vector.
*              ed     - Real array of length (im*idim) containing the B-spline
*                       coefficients of the spline to be approximated.
*              im     - The dimension of the spline space corresponding to et.
*              ilend  - The number of derivatives that are to be kept fixed
*                       at the left end of the spline.
*              irend  - The number of derivatives that are to be kept fixed
*                       at the right end of the spline.
*              ea     - Real array of dimension (im*ik) containing 
*                       the B-spline refinement matrix from the knot vector
*                       etau to the knot vector et. This matrix has
*                       dimension im*in but since at most
*                       ik entries are nonzero in each row, it can
*                       be stored in a im*ik array together
*                       with two integer arrays indicating the position
*                       of the first and last nonzero elements in each
*                       row.
*              inh    - The dimension of the system of normal equations
*                       (inh=in-ilend-irend+1).
*              nfirst - Integer array of dimension (im) containing 
*                       pointers to the first nonzero element of each row 
*                       of the B-spline refinement matrix from etau to et.
*              nlast  - Integer array of dimension (im) containing 
*                       pointers to the last nonzero element of each row 
*                       of the B-spline refinement matrix from etau to et.
*
* 
* OUTPUT     : eb     - Real array of dimension (inh*ik) containing the 
*                       coefficient matrix of the normal equations. This
*                       is really a in*in matrix but since each row of ea
*                       only has at most ik nonzero elements and eb
*                       essentially is of the form eb = ea(tr)*ea, it is
*                       easy to see that eb is a band matrix of band with
*                       at most ik. Since eb also is symmetric it is enough
*                       to store the elements below and on the diagonal.
*                       This is done by storing the diagonal element of row
*                       i in eb(i*ik+ik-1), and then letting d2sta[i] indicate
*                       the position of the first nonzero element in row i 
*                       of eb.
*              ec     - Real array of lengh (in*idim) containing the right
*                       hand side of the normal equations (or really the
*                       idim right hand sides).
*              n2sta  - Integer array of length (in*idim) containing pointers
*                       to the first nonzero elements of the in rows of eb.
*              jstat      - status messages  
*                                         > 0      : warning
*                                         = 0      : ok
*                                         < 0      : error
*             
* 
* METHOD     : 
*
*
* REFERENCES : 
*              
*
* USE        :
*
*-
* CALLS      :   
*
* WRITTEN BY : Vibeke Skytt, SI, 05.92, on the basis of a routine
*              written by Tom Lyche and Knut Moerken, 12.85.
*
*********************************************************************
*/
{ 
  int ki,kj,kr;
  int ki1,ki2,kr1,kj1;
  int kik,kih,krh,kihh,krhh;
  double tw;
  double thelp;
  double *swa = SISL_NULL;
  
  /* Allocate space for a local array of length in to be used during
     multiplication with dtau(-1/2).  */
  
  if ((swa = newarray(in,DOUBLE)) == SISL_NULL)goto err101;
  
  /* Initiate output arrays to zero.  */
  
  for (kj=0; kj<inh; kj++) n2sta[kj] = -1;
  memzero(ec+ilend*idim,inh*idim,DOUBLE);
  memzero(eb,inh*ik,DOUBLE);
  
  /* Determine the normal equations.
     Compute ea(tr)*dt*ea and ea(tr)*dt*ed and store in eb and ec
     respectively.  */
  
  for (kj=0; kj<im; kj++)
    {
       ki1 = MAX(nfirst[kj],ilend);
       ki2 = MIN(nlast[kj],in-irend-1);
       kr1 = ik - ki2 + ki1 - 1;
       tw = (et[kj+ik] - et[kj])/(double)ik;
       kik = ik - 1;
       krhh = ik - nlast[kj] + ki1 - 1;
       for (kr = kr1; kr<ik; ki1++, kik--, krhh++, kr++)
	 {
	    kihh = ki1 - ilend;
	    if (n2sta[kihh] == -1) n2sta[kihh] = kik;
	    kih = ik - nlast[kj] + ki1 - 1;
	    krh = ik - 1;
	    thelp = tw*ea[kj*ik+krhh];
	    for (ki=ki1; ki<=ki2; kih++, kihh++, krh--, ki++)
	      eb[kihh*ik+krh] += ea[kj*ik+kih]*thelp;
	    kih = kr - ik + ki2 + 1;
	    for (ki=0; ki<idim; ki++)
	      ec[kih*idim+ki] += ed[kj*idim+ki]*thelp;
	 }
    }
  
  /* Multiply eb and ec by dtau(-1/2), but remember to skip the first ilen
     and last irend elements of ec and dtau. */
  
  for (kihh=ilend, ki=0; ki<inh; kihh++, ki++)
       swa[ki] = sqrt((double)ik/(etau[kihh+ik]-etau[kihh]));

  for (kihh=ilend, ki=0; ki<inh; kihh++, kih++, ki++)
    {
       thelp = swa[ki];
       for (kj=0; kj<idim; kj++)
	 ec[kihh*idim+kj] *= thelp;
       for (kj1=n2sta[ki], kih=kj1-ik+ki+1, kj=kj1;
	kj<ik; kih++, kj++)
	 eb[ki*ik+kj] *= swa[kih]*thelp;
    }
   

  /* Normal equations set.  */
  
   *jstat = 0;
   goto out;
   
   /* Error in space allocation.  */
   
   err101: *jstat = -101;
   goto out;
   
   out:
      /* Free scratch used for local array.  */
      
      if (swa != SISL_NULL) freearray(swa);
	  
      return;
}
   
