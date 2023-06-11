{//bodyLib scripts
    function fillTextareas() {
        const textareaSurfaceNormals = document.getElementById("surface-normals-input");
        const textareaSurfacePoints = document.getElementById("surface-points-input");
        const textareaAppliedForce = document.getElementById("applied-force-input");
        const textareaAppliedCoordinates = document.getElementById("applied-coordinates-input");
        const textareaDefaultAccuracy = document.getElementById("default-accuracy-input");
        const textareaFrictionCoefficient = document.getElementById("friction-coefficient-input");
        const textareaInitialStiffness = document.getElementById("initial-stiffness-input");
        const textareaForceScanRange = document.getElementById("force-scan-range-input");
        const textareaForceNumPoints = document.getElementById("force-num-points-input");
        const textareaForceAccuracy = document.getElementById("force-accuracy-input");
        const textareaCoordScanRange = document.getElementById("coord-scan-range-input");
        const textareaCoordNumPoints = document.getElementById("coord-num-points-input");
        const textareaCoordAccuracy = document.getElementById("coord-accuracy-input");
      
        textareaSurfaceNormals.value = surfaceNormals.map(row => row.join("\t")).join("\n");
        textareaSurfacePoints.value = surfacePoints.map(row => row.join("\t")).join("\n");
        textareaAppliedForce.value = appliedForce.map(row => row.join("\t")).join("\n");
        textareaAppliedCoordinates.value = appliedCoordinates.map(row => row.join("\t")).join("\n");
        textareaDefaultAccuracy.value = defaultAccuracy;
        textareaFrictionCoefficient.value = frictionCoefficient;
        textareaInitialStiffness.value = initialStiffness.toExponential(2);
        textareaForceScanRange.value = forceScanRange;
        textareaForceNumPoints.value = forceNumPoints;
        textareaForceAccuracy.value = forceAccuracy;
        textareaCoordScanRange.value = coordScanRange;
        textareaCoordNumPoints.value = coordNumPoints;
        textareaCoordAccuracy.value = coordAccuracy;
      }
      
      function updateTextareaValues() {
        surfaceNormals = document.getElementById("surface-normals-input").value.split("\n").map(row => row.split("\t").map(parseFloat));
        surfacePoints = document.getElementById("surface-points-input").value.split("\n").map(row => row.split("\t").map(parseFloat));
        appliedForce = document.getElementById("applied-force-input").value.split("\n").map(row => row.split("\t").map(parseFloat));
        appliedCoordinates = document.getElementById("applied-coordinates-input").value.split("\n").map(row => row.split("\t").map(parseFloat));
        defaultAccuracy = parseFloat(document.getElementById("default-accuracy-input").value);
        frictionCoefficient = parseFloat(document.getElementById("friction-coefficient-input").value);
        initialStiffness = parseFloat(document.getElementById("initial-stiffness-input").value);
        forceScanRange = parseFloat(document.getElementById("force-scan-range-input").value);
        forceNumPoints = parseInt(document.getElementById("force-num-points-input").value);
        forceAccuracy = parseFloat(document.getElementById("force-accuracy-input").value);
        coordScanRange = parseFloat(document.getElementById("coord-scan-range-input").value);
        coordNumPoints = parseInt(document.getElementById("coord-num-points-input").value);
        coordAccuracy = parseFloat(document.getElementById("coord-accuracy-input").value);
      
      /*   console.log(surfaceNormals);
        console.log(surfacePoints);
        console.log(appliedForce);
        console.log(appliedCoordinates);
        console.log(defaultAccuracy);
        console.log(frictionCoefficient);
        console.log(initialStiffness);
        console.log(forceScanRange);
        console.log(forceNumPoints);
        console.log(forceAccuracy);
        console.log(coordScanRange);
        console.log(coordNumPoints);
        console.log(coordAccuracy); */
      }
      



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

function normr(valueIn) {
    const deform = [...valueIn]//.map(row => [...row]); // Create a copy of the array
    for (let i = 0; i < deform.length; i++) {
        const row = deform[i];
        const norm = math.norm(row, 2); // Compute the L2 norm of the row
        if (norm !== 0) { // Check if the norm is not zero
            deform[i] = row.map(x => x / norm); // Divide each element of the row by its norm
        } else {
            deform[i] = row; // If the norm is zero, leave the row unchanged
        }
    }
    return deform;
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

function rowForceB(Fc, coordFc) {
    const b = [0, 0, 0, 0, 0, 0];

    for (let i = 0; i < Fc.length; i++) {
        const rowFc = Fc[i];
        const rowCoordFc = coordFc[i];

        const bRow = [
            -rowFc[0],
            -rowFc[1],
            -rowFc[2],
            rowCoordFc[1] * rowFc[2] - rowCoordFc[2] * rowFc[1],
            rowCoordFc[2] * rowFc[0] - rowCoordFc[0] * rowFc[2],
            rowCoordFc[0] * rowFc[1] - rowCoordFc[1] * rowFc[0]
        ];

        for (let j = 0; j < bRow.length; j++) {
            b[j] += bRow[j];
        }
    }

    return b;
}




function kynCalc(n, p, mu, scale, Fc, coordFc) {

    const normNormals = normr(n)


    //const { norm: _norm, inv, multiply, transpose, subtract, add } = require('mathjs');

    const Ma = motionAinv(n, p); // calculate motion matrix a

    const Mb = motionB(n, scale) // calculate motion matrix b

    const WCSdeform = math.multiply(Ma, Mb); // calculate WCS deformation (motion matrices per retracted interface)

    const PointDeformation = calculateDeformation(p, WCSdeform) // calculate deformation per point
    //console.log(PointDeformation )
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
  
    // Determine the number of columns in the data
    const numColumns = Array.isArray(data[0]) ? data[0].length : 1;
  
    // Create rows for each case with row label and data values
    for (let i = 0; i < rowNames.length; i++) {
      const row = document.createElement("tr");
  
      // Add row name as the first cell
      const rowNameCell = document.createElement("td");
      rowNameCell.style.border = "1px solid black"; // Add border to row name cell
      rowNameCell.appendChild(document.createTextNode(rowNames[i]));
      row.appendChild(rowNameCell);
  
      if (numColumns === 1) {
        // Single column data
        const cell = document.createElement("td");
        if (data[i] < 0) {
          cell.style.color = "red";
        }
        cell.style.border = "1px solid black"; // Add border to data cell
  
        // Format the number based on the specified format
        if (numberFormat === "exponential") {
          cell.appendChild(document.createTextNode(data[i].toExponential(2)));
          
        } else if (numberFormat === "fixed") {
          //cell.appendChild(document.createTextNode(data[i].toFixed(2)));
          cell.appendChild(document.createTextNode(data[i]));
        } else {
          // Default: No specific format, use the number as is
          cell.appendChild(document.createTextNode(data[i]));
        }
  
        row.appendChild(cell);
      } else {
        // Multiple columns data
        for (let j = 0; j < numColumns; j++) {
          const cell = document.createElement("td");
          if (data[i][j] < 0) {
            cell.style.color = "red";
          }
          cell.style.border = "1px solid black"; // Add border to data cells
  
          // Format the number based on the specified format
          if (numberFormat === "exponential") {
            //cell.appendChild(document.createTextNode(data[i][j].toExponential(2)));
            cell.appendChild(document.createTextNode(data[i][j]));
          } else if (numberFormat === "fixed") {
            //cell.appendChild(document.createTextNode(data[i][j].toFixed(2)));
            cell.appendChild(document.createTextNode(data[i][j]));
          } else {
            // Default: No specific format, use the number as is
            cell.appendChild(document.createTextNode(data[i][j]));
          }
  
          row.appendChild(cell);
        }
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
       
  function roundArrayToDecimals(array,dig) {
    return array.map(number => Number(number.toFixed(dig)));
  }
  
  


function findNegativeForceMuPlus(nn, pp, Fcc, coordFcc, resolution) {


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
        mu += resolution;

    }
}

function findNegativeForceMuMin(nn, pp, Fcc, coordFcc, resolution) {

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
        mu -= resolution;
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

function findMinMaxNumbers(array) {
    const smallestNumbers = [];
    const largestNumbers = [];
    for (let i = 0; i < array.length; i++) {
        let smallest = Infinity;
        let largest = -Infinity;

        for (let j = 0; j < array[i].length; j++) {
            const value = array[i][j];

            if (value < smallest) {
                smallest = value;
            }

            if (value > largest) {
                largest = value;
            }
        }

        smallestNumbers.push(smallest);
        largestNumbers.push(largest);
    }

    return [smallestNumbers, largestNumbers];

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
        [0, math.cos(thx), -math.sin(thx), 0],
        [0, math.sin(thx), math.cos(thx), 0],
        [0, 0, 0, 1]
    ];

    const Ry = [
        [math.cos(thy), 0, math.sin(thy), 0],
        [0, 1, 0, 0],
        [-math.sin(thy), 0, math.cos(thy), 0],
        [0, 0, 0, 1]
    ];

    const Rz = [
        [math.cos(thz), -math.sin(thz), 0, 0],
        [math.sin(thz), math.cos(thz), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];

    const Txyz = [
        [1, 0, 0, X],
        [0, 1, 0, Y],
        [0, 0, 1, Z],
        [0, 0, 0, 1]
    ];

    const Tout = math.multiply(math.multiply(math.multiply(math.multiply(math.multiply(Txyz, Tco), Rz), Ry), Rx), Tci);
    return Tout;
}

function concatenateMatrices(matrix1, matrix2) {
    if (matrix1.length !== matrix2.length) {
        throw new Error("Matrix dimensions do not match.");
    }

    const result = [];
    const rows = matrix1.length;

    for (let i = 0; i < rows; i++) {
        result.push([...matrix1[i], ...matrix2[i]]);
    }

    return result;
}


function findMaxPerRow(array) {
    const maxColumns = [];

    for (let row = 0; row < array.length; row++) {
        let max = array[row][0];
        let maxCol = 0;

        for (let col = 1; col < array[row].length; col++) {
            if (array[row][col] > max) {
                max = array[row][col];
                maxCol = col;
            }
        }

        maxColumns.push(maxCol);
    }

    return maxColumns;
}

function perturbFc(nn, pp, Fcc, coordFcc, perturbationRange, xn, resolution) {
    const mumaxvaluePlusValues = [];

    for (let i = 0; i < Fcc.length; i++) {
        const perturbedFcc = [...Fcc];
        const columnValues = [];

        const perturbationStart = Fcc[i] - perturbationRange;
        const perturbationEnd = Fcc[i] + perturbationRange;
        const perturbationStep = (perturbationEnd - perturbationStart) / xn;

        for (let j = 0; j <= xn; j++) {
            const perturbationValue = perturbationStart + j * perturbationStep;
            perturbedFcc[i] = perturbationValue;

            const mumaxvaluePlus = findNegativeForceMuPlus(nn, pp, perturbedFcc, coordFcc, resolution);
            columnValues.push(mumaxvaluePlus);
        }

        mumaxvaluePlusValues.push(columnValues);
    }

    return mumaxvaluePlusValues;
}

function perturbCoordFc(nn, pp, Fcc, coordFcc, perturbationRange, xn, resolution) {
    const mumaxvaluePlusValues = [];

    for (let i = 0; i < coordFcc.length; i++) {
        const perturbedCoordFcc = [...coordFcc];
        const columnValues = [];

        const perturbationStart = coordFcc[i] - perturbationRange;
        const perturbationEnd = coordFcc[i] + perturbationRange;
        const perturbationStep = (perturbationEnd - perturbationStart) / xn;

        for (let j = 0; j <= xn; j++) {
            const perturbationValue = perturbationStart + j * perturbationStep;
            perturbedCoordFcc[i] = perturbationValue;

            const mumaxvaluePlus = findNegativeForceMuPlus(nn, pp, Fcc, perturbedCoordFcc, resolution);
            columnValues.push(mumaxvaluePlus);
        }

        mumaxvaluePlusValues.push(columnValues);
    }

    return mumaxvaluePlusValues;
}

function calcNewSetpoint(start_value, new_value, new_setpoint) {
    const CoordX = start_value[0] + new_value[new_setpoint[0]];
    const CoordY = start_value[1] + new_value[new_setpoint[1]];
    const CoordZ = start_value[2] + new_value[new_setpoint[2]];

    const coordNew = [CoordX, CoordY, CoordZ];
    const formattedCoordNew = coordNew.map((element) => Number(element.toFixed(3)));

    return [formattedCoordNew[0], formattedCoordNew[1], formattedCoordNew[2]];
}

function calculateBoundingBoxSize(coordinates) {
    let minX = coordinates[0][0];
    let maxX = coordinates[0][0];
    let minY = coordinates[0][1];
    let maxY = coordinates[0][1];
    let minZ = coordinates[0][2];
    let maxZ = coordinates[0][2];

    for (let i = 1; i < coordinates.length; i++) {
        const point = coordinates[i];
        const x = point[0];
        const y = point[1];
        const z = point[2];

        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
        minZ = Math.min(minZ, z);
        maxZ = Math.max(maxZ, z);
    }

    const sizeX = maxX - minX;
    const sizeY = maxY - minY;
    const sizeZ = maxZ - minZ;
    const maxDimension = Math.max(sizeX, sizeY, sizeZ);

    return {
        sizeX,
        sizeY,
        sizeZ,
        maxDimension
    };
}

function create3DPlot(normals, points, forces, forceCoordinates) {
    const coordinatesTrace = {
        x: [],
        y: [],
        z: [],
        mode: 'markers',
        marker: {
            size: 5,
            color: 'blue'
        },
        type: 'scatter3d',
        name: 'Points'
    };

    const vectorTraces = []; // Array to hold the vector traces

    for (let i = 0; i < points.length; i++) {
        const [x, y, z] = points[i];
        coordinatesTrace.x.push(x);
        coordinatesTrace.y.push(y);
        coordinatesTrace.z.push(z);

        const [u, v, w] = normals[i];
        const startPoint = [x - u, y - v, z - w]; // Starting point of the vector

        const vectorTrace = {
            x: [startPoint[0], x],
            y: [startPoint[1], y],
            z: [startPoint[2], z],
            mode: 'lines',
            line: {
                color: 'red',
                width: 3
            },
            type: 'scatter3d',
            name: `N ${i + 1}`
        };

        vectorTraces.push(vectorTrace);
    }

    const forceTraces = []; // Array to hold the force traces

    for (let i = 0; i < forces.length; i++) {
        const [forceX, forceY, forceZ] = forces[i];

        const [forceCoordX, forceCoordY, forceCoordZ] = forceCoordinates[i];

        const forceTrace = {
            x: [forceCoordX, forceCoordX + forceX],
            y: [forceCoordY, forceCoordY + forceY],
            z: [forceCoordZ, forceCoordZ + forceZ],
            mode: 'lines',
            line: {
                color: 'green',
                width: 3
            },
            type: 'scatter3d',
            name: `F ${i + 1}`
        };

        forceTraces.push(forceTrace);
    }
    const forceConnectionTrace = {
        x: forceCoordinates.map(coord => coord[0]),
        y: forceCoordinates.map(coord => coord[1]),
        z: forceCoordinates.map(coord => coord[2]),
        mode: 'markers',
        marker: {
            size: 5,
            color: 'green'
        },
        type: 'scatter3d',
        name: 'Force Connection'
    };
    const data = [coordinatesTrace, ...vectorTraces, ...forceTraces, forceConnectionTrace]; // Include coordinate traces, vector traces, and force traces

    const layout = {
        scene: {
            xaxis: {
                scaleanchor: 'y',
                scaleratio: 1
            },
            yaxis: {
                scaleanchor: 'z',
                scaleratio: 1
            },
            zaxis: {
                scaleanchor: 'x',
                scaleratio: 1
            },
            annotations: [{
                x: forces[0][0],
                y: forces[0][1],
                z: forces[0][2],
                showarrow: true,
                arrowhead: 10,
                ax: 0,
                ay: 0,
                az: -40,
            }]
        },


        showlegend: false

    };

    Plotly.newPlot('plotConfig', data, layout);
}


{
    function dotMult(array1, array2) {
        console.log(array1,array2)
        if (array1.length !== array2.length) {
          throw new Error("Arrays must have the same length");
        }
      
        const result = [];
      
        for (let i = 0; i < array1.length; i++) {
          result.push(array1[i] * array2[i]);
        }
      
        return result;
      }
}

{
    function reciprocalArray(array) {
        const result = [];
      
        for (let i = 0; i < array.length; i++) {
          const reciprocal = 1 / array[i];
          result.push(reciprocal);
        }
      
        return result;
      }
}

{
    function combineArrays(...arrays) {
        const combinedArray = [];
      
        for (let i = 0; i < arrays[0].length; i++) {
          const row = arrays.map(array => array[i]);
          combinedArray.push(row);
        }
      
        return combinedArray;
      }
}
}