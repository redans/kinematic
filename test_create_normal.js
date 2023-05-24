const Math = require('mathjs');

function cTransL(X, Y, Z, thx, thy, thz, cX, cY, cZ) {
        const Tci = [
          [1, 0, 0, -cX],
          [0, 1, 0, -cY],
          [0, 0, 1, -cZ],
          [0, 0, 0, 1]
        ];
    
        const Tco = [
          [1, 0, 0, cX],
          [0, 1, 0, cY],
          [0, 0, 1, cZ],
          [0, 0, 0, 1]
        ];
    
        const Rx = [
          [1, 0, 0, 0],
          [0, Math.cos(thx), -Math.sin(thx), 0],
          [0, Math.sin(thx), Math.cos(thx), 0],
          [0, 0, 0, 1]
        ];
    
        const Ry = [
          [Math.cos(thy), 0, Math.sin(thy), 0],
          [0, 1, 0, 0],
          [-Math.sin(thy), 0, Math.cos(thy), 0],
          [0, 0, 0, 1]
        ];
    
        const Rz = [
          [Math.cos(thz), -Math.sin(thz), 0, 0],
          [Math.sin(thz), Math.cos(thz), 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1]
        ];
    
        const Txyz = [
          [1, 0, 0, X],
          [0, 1, 0, Y],
          [0, 0, 1, Z],
          [0, 0, 0, 1]
        ];
    
        const Tout = Math.multiply(Math.multiply(Math.multiply(Math.multiply(Txyz, Tco), Rz), Ry), Rx, Tci);
    
        return Tout;
      }


        const n = [0, 0, 1, 0];
        const p1 = [-20e-3, -0.5, 0, 1];
        const p2 = [20e-3, -0.5, 0, 1];

        beta = Math.PI / 180 * 60;
        alpha = 0;
        gamma = 0;
        const N1 = cTransL(0, 0, 0, alpha, beta, gamma, 0, 0, 0) * n;
        const N2 = cTransL(0, 0, 0, alpha, -beta, gamma, 0, 0, 0) * n;
        const P1 = cTransL(0, 0, 0, alpha, beta, gamma, 0, 0, 0) * p1;
        const P2 = cTransL(0, 0, 0, alpha, -beta, gamma, 0, 0, 0) * p2;

        const G1p = [P1, P2];
        const G1n = [N1, N2];

        const G2p = cTransL(0, 0, 0, 0, 0, Math.PI / 180 * 120, 0, 0, 0) * G1p;
        const G2n = cTransL(0, 0, 0, 0, 0, Math.PI / 180 * 120, 0, 0, 0) * G1n;

        const G3p = cTransL(0, 0, 0, 0, 0, -Math.PI / 180 * 120, 0, 0, 0) * G1p;
        const G3n = cTransL(0, 0, 0, 0, 0, -Math.PI / 180 * 120, 0, 0, 0) * G1n;

        const predefinedNormals = [G1n, G2n, G3n];

        const predefinedPoints = [G1p, G2p, G3p];

        console.log(predefinedNormals)


