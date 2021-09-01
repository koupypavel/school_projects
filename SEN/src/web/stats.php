<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);
$host               = "localhost";
$username           = "root";
$password           = "mrkev1234";
$dbname             = "sen_iot";
$dataPointsTemp     = array();
$dataPointsHumidity = array();
$dataPointsVolume   = array();
try {
$pdo     = new PDO("mysql:host=$host;dbname=$dbname", $username, $password);
$sqlTemp = 'SELECT * FROM sen_iot.data';
$q       = $pdo->query($sqlTemp);
$q->setFetchMode(PDO::FETCH_ASSOC);
while ($row = $q->fetch()) {
if ($row['quantity'] == "temp") {
array_push($dataPointsTemp, array("y" => intval($row['value']), "label" => $row['time']));
} elseif ($row['quantity'] == "humid") {
array_push($dataPointsHumidity, array("y" => intval($row['value']), "label" => $row['time']));
} elseif ($row['quantity'] == "volume") {
array_push($dataPointsVolume, array("y" => intval($row['value']), "label" => $row['time']));
}
}
} catch (PDOException $e) {
die("Could not connect to the database $dbname :" . $e->getMessage());
}
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
        legendText: "Teplota [°C]",
        dataPoints: <?php echo json_encode($dataPointsTemp, JSON_NUMERIC_CHECK); ?>
        },
        {
        type: "line",
        showInLegend: true,
        legendText: "Vlhkost [%]",
        dataPoints: <?php echo json_encode($dataPointsHumidity, JSON_NUMERIC_CHECK); ?>
        },
        {
        type: "line",
        showInLegend: true,
        legendText: "Množství vody [l]",
        dataPoints: <?php echo json_encode($dataPointsVolume, JSON_NUMERIC_CHECK); ?>
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