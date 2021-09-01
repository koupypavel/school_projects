<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);
$host               = "localhost";
$username           = "nav";
$password           = "polda2541";
$dbname             = "hexiwear";
$dataPointsTemp     = array();
$dataPointsHumidity = array();
$dataPointsVolume   = array();
//todo
?>
<!DOCTYPE HTML>
<html>
    <head>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.1/css/bootstrap.min.css">
        <link rel="stylesheet" type="text/css" href="style.css?v=12345">
        <style type="text/css">
        .btn {
        margin-top: 280px;
        margin-left: 10px;
        }
        </style>
        <script>
        window.onload = function () {
        var chartTemp = new CanvasJS.Chart("chartContainerTemp", {
        height: 270,
        axisY: {
        title: ""
        },
        data: [
        {
        type: "line",
        showInLegend: true,
        legendText: "Srdeční tep",
        dataPoints: <?php echo json_encode($dataPointsTemp, JSON_NUMERIC_CHECK); ?>
        }]
        });
        chartTemp.render();
        }
        </script>
    </head>
    <body>
        <div id="chartContainerTemp" style="height: 100; width: 100%;"></div>
        <script src="https://canvasjs.com/assets/script/canvasjs.min.js"></script>
        <a href="index.php">
            <button type="button" class="btn btn-danger">
            Zpět
            </button>
        </a>
    </body>
</html>