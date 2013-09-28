/*
C interface to libANN for linking with python ctypes.

Copyright (C) 2008, Robert Hetland

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.
3. The name of the author may not be used to endorse or promote
   products derived from this software without specific prior written
   permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <ANN/ANN.h>
#include <ANN/ANNperf.h>

extern "C" void* init_kd_tree(double *pts1, int nPts, int dim, int bs, ANNsplitRule split)
{   
    int i;
    ANNkd_tree *kdTree;
    
    /// Convert (*double) input array into a **double
    double** pts = (double **)malloc(nPts * sizeof(double *));

    for (i=0; i<nPts; i++) {
        pts[i] = pts1 + i*dim;
    }
    
    
    /// Initialize and return new kd_tree object.
    kdTree = new ANNkd_tree(pts, nPts, dim, bs, split);
    
    return kdTree;
}

extern "C" void kd_tree_search(ANNkd_tree *kdTree, 
                               ANNpoint qpts, int nPts,
                               int k, double eps,
                               int *nnIdx, double *dists) {
    
    int dim = kdTree -> theDim();
    for( int i = 0; i < nPts; i++) { /// itterate over query points.
        kdTree -> annkSearch(&qpts[dim*i], k, &nnIdx[k*i], &dists[k*i], eps); 
    }
}


extern "C" void kd_tree_priority_search(ANNkd_tree *kdTree, 
                                        ANNpoint qpts, int nPts,
                                        int k, double eps,
                                        int *nnIdx, double *dists) {
    
    int dim = kdTree -> theDim();
    for( int i = 0; i < nPts; i++) { /// itterate over query points.
        kdTree -> annkPriSearch(&qpts[dim*i], k,
                                &nnIdx[k*i], &dists[k*i], eps); 
    }
}


extern "C" void kd_tree_fixed_radius_search(ANNkd_tree *kdTree,
                                            ANNdist sqRad,
                                            ANNpoint qpts, int nPts,
                                            int k, double eps,
                                            int *nnIdx, double *dists) {
    
    int dim = kdTree -> theDim();
    for( int i = 0; i < nPts; i++) { /// itterate over query points.
        kdTree -> annkFRSearch(&qpts[dim*i], sqRad, k, 
                               &nnIdx[k*i], &dists[k*i], eps); 
    }
}


extern "C" void kd_tree_print(ANNkd_tree *kdTree) {
    kdTree -> Print(ANNtrue, std::cout);
}


typedef struct {
    int n_lf, n_tl, n_spl, depth;
    double sum_ar, avg_ar;
} statistics;


extern "C" void kd_tree_get_stats(ANNkd_tree *kdTree, 
                                  statistics *kd_tree_stats) {
    ANNkdStats st;
    kdTree -> getStats(st);
    
    kd_tree_stats->n_lf = st.n_lf;
    kd_tree_stats->n_tl = st.n_tl;
    kd_tree_stats->n_spl = st.n_spl;
    kd_tree_stats->depth = st.depth;
    
    kd_tree_stats->sum_ar = st.sum_ar;
    kd_tree_stats->avg_ar = st.avg_ar;
}


extern "C" void kd_tree_delete(ANNkd_tree *kdTree) {
    if( kdTree != NULL ) {
        double** pts = kdTree->thePoints();
        delete kdTree;
        annClose();
        free(pts);
    }
}
