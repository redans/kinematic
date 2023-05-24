{ //kynlib
  function motionAinv(n, p) {

    const a = [[n[0][0], n[0][1], n[0][2], n[0][2] * p[0][1] - n[0][1] * p[0][2], n[0][0] * p[0][2] - n[0][2] * p[0][0], n[0][1] * p[0][0] - n[0][0] * p[0][1]],
    [n[1][0], n[1][1], n[1][2], n[1][2] * p[1][1] - n[1][1] * p[1][2], n[1][0] * p[1][2] - n[1][2] * p[1][0], n[1][1] * p[1][0] - n[1][0] * p[1][1]],
    [n[2][0], n[2][1], n[2][2], n[2][2] * p[2][1] - n[2][1] * p[2][2], n[2][0] * p[2][2] - n[2][2] * p[2][0], n[2][1] * p[2][0] - n[2][0] * p[2][1]],
    [n[3][0], n[3][1], n[3][2], n[3][2] * p[3][1] - n[3][1] * p[3][2], n[3][0] * p[3][2] - n[3][2] * p[3][0], n[3][1] * p[3][0] - n[3][0] * p[3][1]],
    [n[4][0], n[4][1], n[4][2], n[4][2] * p[4][1] - n[4][1] * p[4][2], n[4][0] * p[4][2] - n[4][2] * p[4][0], n[4][1] * p[4][0] - n[4][0] * p[4][1]],
    [n[5][0], n[5][1], n[5][2], n[5][2] * p[5][1] - n[5][1] * p[5][2], n[5][0] * p[5][2] - n[5][2] * p[5][0], n[5][1] * p[5][0] - n[5][0] * p[5][1]],
    ];
    return math.inv(a);
  }

  function motionB(n, scale) {
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

  function calculateDeformation(p, DOF) {
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
    const af = [[F[0][0], F[1][0], F[2][0], F[3][0], F[4][0], F[5][0]],
    [F[0][1], F[1][1], F[2][1], F[3][1], F[4][1], F[5][1]],
    [F[0][2], F[1][2], F[2][2], F[3][2], F[4][2], F[5][2]],
    [F[0][1] * p[0][2] - F[0][2] * p[0][1], F[1][1] * p[1][2] - F[1][2] * p[1][1], F[2][1] * p[2][2] - F[2][2] * p[2][1], F[3][1] * p[3][2] - F[3][2] * p[3][1], F[4][1] * p[4][2] - F[4][2] * p[4][1], F[5][1] * p[5][2] - F[5][2] * p[5][1]],
    [F[0][2] * p[0][0] - F[0][0] * p[0][2], F[1][2] * p[1][0] - F[1][0] * p[1][2], F[2][2] * p[2][0] - F[2][0] * p[2][2], F[3][2] * p[3][0] - F[3][0] * p[3][2], F[4][2] * p[4][0] - F[4][0] * p[4][2], F[5][2] * p[5][0] - F[5][0] * p[5][2]],
    [F[0][0] * p[0][1] - F[0][1] * p[0][0], F[1][0] * p[1][1] - F[1][1] * p[1][0], F[2][0] * p[2][1] - F[2][1] * p[2][0], F[3][0] * p[3][1] - F[3][1] * p[3][0], F[4][0] * p[4][1] - F[4][1] * p[4][0], F[5][0] * p[5][1] - F[5][1] * p[5][0]]
    ];

    return af;
  }

  function ForceB(Fc, coordFc) {
    const b = [
      -Fc[0],
      -Fc[1],
      -Fc[2],
      coordFc[1] * Fc[2] - coordFc[2] * Fc[1],
      coordFc[2] * Fc[0] - coordFc[0] * Fc[2],
      coordFc[0] * Fc[2] - coordFc[1] * Fc[0]
    ];

    return b;
  }


  function kynCalc(n, p, mu, scale, Fc, coordFc) {
    //const { norm: _norm, inv, multiply, transpose, subtract, add } = require('mathjs');

    const Ma = motionAinv(n, p); // calculate motion matrix a

    const Mb = motionB(n, scale) // calculate motion matrix b

    const WCSdeform = math.multiply(Ma, Mb); // calculate WCS deformation (motion matrices per retracted interface)

    const PointDeformation = calculateDeformation(p, WCSdeform) // calculate deformation per point

    const normFw = normrP(PointDeformation); // normalized friction vectors



    const F = normrP(math.add(math.multiply(normFw, mu), n)) // normalized force vectors 
    //console.table('stop')
    // calculate all 6 force matrices a
    const CC = [];
    for (var i = 0; i <= 5; i++) {
      CC.push(ForceA(p, F[i]));
    }

    const BB = ForceB(Fc, coordFc) // calculate force vector b

    // calculate interface forces
    const Finterface = [];
    for (var i = 0; i <= 5; i++) {
      Finterface.push(math.multiply(math.inv(CC[i]), BB));
    }

    //console.table(transpose(Finterface))

    return [Finterface, WCSdeform]
  }


  function populateTable(tableId, data, fontSize, numberFormat, rowNames, columnNames, tableHeader) {
    const table = document.getElementById(tableId);
    table.style.fontSize = fontSize;
    table.innerHTML = ""; // Clear the table
    table.style.borderCollapse = "collapse"; // Add border-collapse style
    table.style.border = "2px solid black"; // Add border style

    // Create table header (caption)
    if (tableHeader) {
      const caption = document.createElement("caption");
      const captionText = document.createTextNode(tableHeader);
      caption.appendChild(captionText);
      caption.style.fontSize = "14px"; // Set font size
      caption.style.fontWeight = "bold"; // Apply bold style
      table.appendChild(caption);
    }

    // Create header row with column names
    const headerRow = document.createElement("tr");
    const emptyHeaderCell = document.createElement("th"); // Empty header cell for row labels
    headerRow.appendChild(emptyHeaderCell);

    for (let i = 0; i < columnNames.length; i++) {
      const cell = document.createElement("th");
      cell.style.border = "1px solid black"; // Add border to column name cells
      cell.appendChild(document.createTextNode(columnNames[i]));
      headerRow.appendChild(cell);
    }
    table.appendChild(headerRow);

    // Create rows for each case with row label and data values
    for (let i = 0; i < data.length; i++) {
      const row = document.createElement("tr");

      // Add row name as the first cell
      const rowNameCell = document.createElement("td");
      rowNameCell.style.border = "1px solid black"; // Add border to row name cell
      rowNameCell.appendChild(document.createTextNode(rowNames[i]));
      row.appendChild(rowNameCell);

      for (let j = 0; j < data[i].length; j++) {
        const cell = document.createElement("td");
        if (data[i][j] < 0) {
          cell.style.color = "red";
        }
        cell.style.border = "1px solid black"; // Add border to data cells

        // Format the number based on the specified format
        if (numberFormat === "exponential") {
          cell.appendChild(document.createTextNode(data[i][j].toExponential(1)));
        } else if (numberFormat === "fixed") {
          cell.appendChild(document.createTextNode(data[i][j].toFixed(2)));
        } else {
          // Default: No specific format, use the number as is
          cell.appendChild(document.createTextNode(data[i][j]));
        }

        row.appendChild(cell);
      }
      table.appendChild(row);
    }

    // Apply border styles to entire table
    const cells = table.getElementsByTagName("td");
    for (let i = 0; i < cells.length; i++) {
      cells[i].style.border = "1px solid black";
    }
    const headerCells = table.getElementsByTagName("th");
    for (let i = 0; i < headerCells.length; i++) {
      headerCells[i].style.border = "1px solid black";
    }
  }






  /* function populateTable(tableId, data, fontSize, numberFormat, rowNames, columnNames) {
    const table = document.getElementById(tableId);
    table.style.fontSize = fontSize;
    table.innerHTML = ""; // Clear the table
    table.style.borderCollapse = "collapse"; // Add border-collapse style
    
    table.style.border = "2px solid black"; // Add border style
  
    // Create header row with column names
    const headerRow = document.createElement("tr");
    const emptyCell = document.createElement("td"); // Empty cell for row labels
    emptyCell.style.border = "1px solid black"; // Add border to empty cell
    headerRow.appendChild(emptyCell);
    for (let i = 0; i < columnNames.length; i++) {
      
      const cell = document.createElement("td");
      cell.style.border = "1px solid black"; // Add border to column name cells
      cell.appendChild(document.createTextNode(columnNames[i]));
      headerRow.appendChild(cell);
    }
    table.appendChild(headerRow);
  
    // Create rows for each case with row label and data values
    for (let i = 0; i < data.length; i++) {
      const row = document.createElement("tr");
  
      // Add row name as the first cell
      const rowNameCell = document.createElement("td");
      rowNameCell.style.border = "1px solid black"; // Add border to row name cell
      rowNameCell.appendChild(document.createTextNode(rowNames[i]));
      row.appendChild(rowNameCell);
  
      for (let j = 0; j < data[i].length; j++) {
        const cell = document.createElement("td");
        if (data[i][j] < 0) {
          cell.style.color = "red";
        }
        cell.style.border = "1px solid black"; // Add border to data cells
  
        // Format the number based on the specified format
        if (numberFormat === "exponential") {
          cell.appendChild(document.createTextNode(data[i][j].toExponential(1)));
        } else if (numberFormat === "fixed") {
          cell.appendChild(document.createTextNode(data[i][j].toFixed(2)));
        } else {
          // Default: No specific format, use the number as is
          cell.appendChild(document.createTextNode(data[i][j]));
        }
  
        row.appendChild(cell);
      }
      table.appendChild(row);
    }
  }
   */




  function findNegativeForceMuPlus(nn, pp, Fcc, coordFcc) {
    let mu = 0;
    while (true) {
      // Calculate the forces and deformations for the current mu value
      const [FinterfaceF, WCSdeformF] = kynCalc(nn, pp, mu, 0.1, Fcc, coordFcc);

      // Check if any of the FinterfaceF values are negative
      if (FinterfaceF.some((row) => row.some((val) => val < 0))) {
        // Negative force found, return the current mu value
        return mu;
      }

      // Increment the mu value and continue looping
      mu += 0.005;
    }
  }

  function findNegativeForceMuMin(nn, pp, Fcc, coordFcc) {
    let mu = 0;
    while (true) {
      // Calculate the forces and deformations for the current mu value
      const [FinterfaceF, WCSdeformF] = kynCalc(nn, pp, mu, 0.1, Fcc, coordFcc);

      // Check if any of the FinterfaceF values are negative
      if (FinterfaceF.some((row) => row.some((val) => val < 0))) {
        // Negative force found, return the current mu value
        return mu;
      }

      // Increment the mu value and continue looping
      mu -= 0.005;
    }
  }

  function findLargestNumbers(array) {
    const largestPositiveNumbers = [];
    const largestNegativeNumbers = [];

    for (let i = 0; i < array.length; i++) {
      let maxPositive = -Infinity;
      let maxNegative = -Infinity;

      for (let j = 0; j < array[i].length; j++) {
        const value = array[i][j];

        if (value > 0 && value > maxPositive) {
          maxPositive = value;
        } else if (value < 0 && value > maxNegative) {
          maxNegative = value;
        }
      }

      largestPositiveNumbers.push(maxPositive);
      largestNegativeNumbers.push(maxNegative);
    }

    return [largestPositiveNumbers, largestNegativeNumbers];
  }


  function flipLR(matrix) {
    const flippedMatrix = [];

    for (let i = 0; i < matrix.length; i++) {
      flippedMatrix.push(matrix[i].reverse());
    }

    return flippedMatrix;
  }


  function plotGraph(x, y, targetElementId, title, xAxisName, yAxisName, traceNames) {
    // Create an array to hold the data traces
    var data = [];

    // Iterate over the y array to create individual traces
    for (var i = 0; i < y.length; i++) {
      var trace = {
        x: x,
        y: y[i],
        mode: 'lines',
        name: traceNames[i] || 'Trace ' + (i + 1)
      };
      data.push(trace);
    }

    // Set up the layout options
    var layout = {
      title: title || 'Plot',
      xaxis: { title: xAxisName || 'X' },
      yaxis: { title: yAxisName || 'Y' }
    };

    // Create the plot
    Plotly.newPlot(targetElementId, data, layout);
  }




  function linspace(start, end, numSteps) {
    const stepSize = (end - start) / (numSteps - 1);
    const values = [];

    for (let i = 0; i < numSteps; i++) {
      const value = start + stepSize * i;
      values.push(value);
    }

    return values;
  }

  function diff(data) {
    const result = [];

    for (let i = 0; i < data.length; i++) {
      const line = data[i];
      const diffLine = [];

      for (let j = 1; j < line.length; j++) {
        const diffValue = line[j] - line[j - 1];
        diffLine.push(diffValue);
      }

      result.push(diffLine);
    }

    return result;
  }


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

    const Tout = math.multiply(math.multiply(math.multiply(math.multiply(Txyz, Tco), Rz), Ry), Rx, Tci);

    return Tout;
  }

  function multiplyMatrices(...matrices) {
    return matrices.reduce((result, matrix) => {
      const rows = result.length;
      const cols = matrix[0].length;
      const sharedDim = matrix.length;

      const product = Array.from({ length: rows }, () => Array(cols).fill(0));

      for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
          for (let k = 0; k < sharedDim; k++) {
            product[i][j] += result[i][k] * matrix[k][j];
          }
        }
      }

      return product;
    });
  }






}

