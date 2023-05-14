
function motionAinv(n, p) {
   
   const a= [    [n[0][0], n[0][1], n[0][2], n[0][2]*p[0][1] - n[0][1]*p[0][2], n[0][0]*p[0][2] - n[0][2]*p[0][0], n[0][1]*p[0][0] - n[0][0]*p[0][1]],
      [n[1][0], n[1][1], n[1][2], n[1][2]*p[1][1] - n[1][1]*p[1][2], n[1][0]*p[1][2] - n[1][2]*p[1][0], n[1][1]*p[1][0] - n[1][0]*p[1][1]],
      [n[2][0], n[2][1], n[2][2], n[2][2]*p[2][1] - n[2][1]*p[2][2], n[2][0]*p[2][2] - n[2][2]*p[2][0], n[2][1]*p[2][0] - n[2][0]*p[2][1]],
      [n[3][0], n[3][1], n[3][2], n[3][2]*p[3][1] - n[3][1]*p[3][2], n[3][0]*p[3][2] - n[3][2]*p[3][0], n[3][1]*p[3][0] - n[3][0]*p[3][1]],
      [n[4][0], n[4][1], n[4][2], n[4][2]*p[4][1] - n[4][1]*p[4][2], n[4][0]*p[4][2] - n[4][2]*p[4][0], n[4][1]*p[4][0] - n[4][0]*p[4][1]],
      [n[5][0], n[5][1], n[5][2], n[5][2]*p[5][1] - n[5][1]*p[5][2], n[5][0]*p[5][2] - n[5][2]*p[5][0], n[5][1]*p[5][0] - n[5][0]*p[5][1]],
    ];
  return math.inv(a);  
}

function motionB(n,scale) {
    let b = new Array(6); // create an array of length 6
    for (let i = 0; i < 6; i++) {
      b[i] = new Array(6).fill(0); // initialize each element of b to an array of length 6 filled with 0s
    }
  
    for (let ii = 0; ii < 6; ii++) {
      b[ii][ii] = -scale * math.pow(n[ii][0], 2) - scale * math.pow(n[ii][1], 2) - scale * math.pow(n[ii][2], 2);
    }
    
    return b;
}

function Rt(Rx, Ry, Rz) {
    return [
      [1, -Rz, Ry],
      [Rz, 1, -Rx],
      [-Ry, Rx, 1]
    ];
}


function normrP(deform) {
  for (let i = 0; i < deform.length; i++) {
    const slice = deform[i];
    for (let j = 0; j < slice.length; j++) {
      const row = slice[j];
      const norm = math.norm(row, 2); // Compute the L2 norm of the row
      if (norm !== 0) { // Check if the norm is not zero
        slice[j] = row.map(x => x / norm); // Divide each element of the row by its norm
      } else {
        slice[j] = row; // If the norm is zero, leave the row unchanged
      }
    }
  }
  return deform;
}

function calculateDeformation(p,  DOF) {
  var deformation = [];
 //console.log(p)
 //console.log(DOF)
  const pPrime = math.transpose(p);
  
  for (var i = 0; i <= 5; i++) {
    const temp = math.subtract(math.transpose(p), math.add(math.multiply(Rt(DOF[3][i], DOF[4][i], DOF[5][i]), pPrime), math.transpose([[DOF[0][i], DOF[1][i], DOF[2][i]]])));
    deformation.push(math.transpose(temp.valueOf()));
  }
  //console.log(deformation)
  return deformation;
}


function ForceA(p, F) {
 const af = [    [F[0][0], F[1][0], F[2][0], F[3][0], F[4][0], F[5][0]],
    [F[0][1], F[1][1], F[2][1], F[3][1], F[4][1], F[5][1]],
    [F[0][2], F[1][2], F[2][2], F[3][2], F[4][2], F[5][2]],
    [F[0][1]*p[0][2] - F[0][2]*p[0][1], F[1][1]*p[1][2] - F[1][2]*p[1][1], F[2][1]*p[2][2] - F[2][2]*p[2][1], F[3][1]*p[3][2] - F[3][2]*p[3][1], F[4][1]*p[4][2] - F[4][2]*p[4][1], F[5][1]*p[5][2] - F[5][2]*p[5][1]],
    [F[0][2]*p[0][0] - F[0][0]*p[0][2], F[1][2]*p[1][0] - F[1][0]*p[1][2], F[2][2]*p[2][0] - F[2][0]*p[2][2], F[3][2]*p[3][0] - F[3][0]*p[3][2], F[4][2]*p[4][0] - F[4][0]*p[4][2], F[5][2]*p[5][0] - F[5][0]*p[5][2]],
    [F[0][0]*p[0][1] - F[0][1]*p[0][0], F[1][0]*p[1][1] - F[1][1]*p[1][0], F[2][0]*p[2][1] - F[2][1]*p[2][0], F[3][0]*p[3][1] - F[3][1]*p[3][0], F[4][0]*p[4][1] - F[4][1]*p[4][0], F[5][0]*p[5][1] - F[5][1]*p[5][0]]
  ];

  return af;
}

function ForceB(Fc, coordFc) {
  const b = [
    -Fc[0],
    -Fc[1],
    -Fc[2],
    coordFc[1]*Fc[2] - coordFc[2]*Fc[1],
    coordFc[2]*Fc[0] - coordFc[0]*Fc[2],
    coordFc[0]*Fc[2] - coordFc[1]*Fc[0]
  ];

  return b;
}


function kynCalc(n,p,mu,scale, Fc, coordFc) {
  //const { norm: _norm, inv, multiply, transpose, subtract, add } = require('mathjs');

  const Ma = motionAinv(n, p); // calculate motion matrix a

  const Mb=motionB(n,scale) // calculate motion matrix b
  
  const WCSdeform = math.multiply(Ma, Mb); // calculate WCS deformation (motion matrices per retracted interface)

  const PointDeformation=calculateDeformation(p,WCSdeform) // calculate deformation per point

  const normFw = normrP(PointDeformation); // normalized friction vectors

  
  
  const F=normrP(math.add(math.multiply(normFw,mu),n)) // normalized force vectors 
  console.table('stop')
  // calculate all 6 force matrices a
  const CC = [];
  for (var i = 0; i <= 5; i++) {
  CC.push(ForceA(p, F[i]));
  }

  const BB=ForceB(Fc, coordFc) // calculate force vector b

  // calculate interface forces
  const Finterface = [];
  for (var i = 0; i <= 5; i++) {
  Finterface.push(math.multiply(math.inv(CC[i]), BB));
}

  //console.table(transpose(Finterface))

  return [Finterface, WCSdeform]
}


