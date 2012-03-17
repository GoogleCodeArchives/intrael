function Perspective(){
}

Perspective.prototype = {

srcmatrix : [0.0,0.0,1.0,0.0,0.0,1.0,1.0,1.0],
dstmatrix : [0.0,0.0,1.0,0.0,0.0,1.0,1.0,1.0],
dstdots : [[0.0,0.0],[1.0,0.0],[0.0,1.0],[1.0,1.0]],
srcdots : [[0.0,0.0],[1.0,0.0],[0.0,1.0],[1.0,1.0]],
setOffsets:function(x,y){
    this.offsetX=x;
    this.offsetY=y;
},
process: function(x,y){
               mat = this.warpmatrix;
        
        a = (x * mat[0] + y*mat[4] + mat[12]);
            b = (x * mat[1] + y*mat[5] + mat[13]);
            c = (x * mat[3] + y*mat[7] + mat[15]);  
            return [parseInt(a/c), parseInt(b/c)];
            
      
      },
    setsrc: function(dot1,dot2,dot3,dot4){
        
        this.srcdots = [[parseFloat(dot1[0]),parseFloat(dot1[1])],[parseFloat(dot2[0]),parseFloat(dot2[1])],[parseFloat(dot3[0]),parseFloat(dot3[1])],[parseFloat(dot4[0]),parseFloat(dot4[1])]];
        
        this.computeWarpMatrix();
    },
        
    setdst:function(dot1,dot2,dot3,dot4){
        this.dstdots = [[parseFloat(dot1[0]),parseFloat(dot1[1])],[parseFloat(dot2[0]),parseFloat(dot2[1])],[parseFloat(dot3[0]),parseFloat(dot3[1])],[parseFloat(dot4[0]),parseFloat(dot4[1])]];
        this.computeWarpMatrix();
    },
        
    computeWarpMatrix: function(){
        this.srcmatrix = this.computeQuadToSquare(this.srcdots);
        this.dstmatrix = this.computeSquareToQuad(this.dstdots);
        this.warpmatrix = this.multMats(this.srcmatrix,this.dstmatrix);
    },
        
    computeSquareToQuad: function(inputdots){
        var x0 = inputdots[0][0],
        y0 = inputdots[0][1],
        x1 = inputdots[1][0],
        y1 = inputdots[1][1],
        x2 = inputdots[2][0],
        y2 = inputdots[2][1],
        x3 = inputdots[3][0],
        y3 = inputdots[3][1];
        var dx1 = x1 - x2,
        dy1 = y1 - y2,
        dx2 = x3 - x2,
        dy2 = y3 - y2,
        sx = x0 - x1 + x2 - x3,
        sy = y0 - y1 + y2 - y3;
        var g = (sx * dy2 - dx2 * sy) / (dx1 * dy2 - dx2 * dy1),
        h = (dx1 * sy - sx * dy1) / (dx1 * dy2 - dx2 * dy1),
        a = x1 - x0 + g * x1,
        b = x3 - x0 + h * x3,
        c = x0,
        d = y1 - y0 + g * y1,
        e = y3 - y0 + h * y3,
        f = y0;
        
        var mat = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
            
        mat[ 0] = a;
        mat[ 1] = d;
        mat[ 2] = 0;
        mat[ 3] = g;
            mat[ 4] = b;
        mat[ 5] = e;
        mat[ 6] = 0;
        mat[ 7] = h;
            mat[ 8] = 0;
        mat[ 9] = 0;
        mat[10] = 1;
        mat[11] = 0;
            mat[12] = c;
        mat[13] = f;    
        mat[14] = 0;
        mat[15] = 1;
        return mat;
    },
        
    computeQuadToSquare:function(inputdots){
        var x0 = inputdots[0][0],
        y0 = inputdots[0][1],
        x1 = inputdots[1][0],
        y1 = inputdots[1][1],
        x2 = inputdots[2][0],
        y2 = inputdots[2][1],
        x3 = inputdots[3][0],
        y3 = inputdots[3][1],
        mat = this.computeSquareToQuad(inputdots);
        
        var a = mat[ 0],
        d = mat[ 1],
        g = mat[ 3],
        b = mat[ 4],
        e = mat[ 5],
        h = mat[ 7],            
            c = mat[12],
        f = mat[13];
        
        var A = e - f * h,
            B = c * h - b,
            C = b * f - c * e,
            D = f * g - d,
            E =     a - c * g,
            F = c * d - a * f,
            G = d * h - e * g,
            H = b * g - a * h,
            I = a * e - b * d;
        var idet = 1.0 / (a * A           + b * D           + c * G);
         
        mat[ 0] = A * idet,
        mat[ 1] = D * idet,
        mat[ 2] = 0,
        mat[ 3] = G * idet,
        
            mat[ 4] = B * idet,
        mat[ 5] = E * idet,
        mat[ 6] = 0,
        mat[ 7] = H * idet,
        
            mat[ 8] = 0,       
        mat[ 9] = 0,      
        mat[10] = 1,
        mat[11] = 0,      
        
            mat[12] = C * idet,
        mat[13] = F * idet,
        mat[14] = 0,
        mat[15] = I * idet;
        return mat;
        },
    multMats:function (srcMat,dstMat){
        var resMat = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
        for( var r=0;r<4;r++){
            var ri = r * 4;
            for(var c=0;c<4;c++){
                resMat[ri + c] = srcMat[ri    ] * dstMat[c     ] + srcMat[ri + 1] * dstMat[c +  4] +    srcMat[ri + 2] * dstMat[c +  8] + srcMat[ri + 3] * dstMat[c + 12];
            }
        }
        return resMat;
    }

}
