<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);

$host     = "localhost";
$username = "root";
$password = "mrkev1234";
$dbname   = "sen_iot";

$volume    = "0";
$distance  = "0";
$tankLevel = 50;
$relay1    = 0;
$relay2    = 0;

$avg_level = 0;
$min_level = 0;

function connect($r, $message)
{
    echo "";
}
function subscribe()
{
    echo "";
}
function unsubscribe()
{
    echo "";
}
function message($message)
{
    global $volume, $distance, $relay1, $relay2, $tankLevel;
//printf("Got a message on topic %s with payload:\n%s\n", $message->topic, $message->payload);
    if (strcmp($message->topic, "uwls/raw/dist") == 0) {
        $distance = $message->payload;
    } elseif (strcmp($message->topic, "uwls/calc/volume") == 0) {
        $volume = $message->payload;
    } elseif (strcmp($message->topic, "uwls/cmd/relay1") == 0) {
        $relay1 = intval($message->payload);
    } elseif (strcmp($message->topic, "uwls/cmd/relay2") == 0) {
        $relay2 = intval($message->payload);
    } elseif (strcmp($message->topic, "uwls/calc/tankLevel") == 0) {
        $tankLevel = intval($message->payload);
    }

    echo "";
}
function disconnect()
{
    echo "";
}
function logger()
{
    echo ""; //var_dump(func_get_args());
}

$array_wl = array();
try {
    $pdo = new PDO("mysql:host=$host;dbname=$dbname", $username, $password);

    $sqltemp = 'SELECT * FROM sen_iot.data  WHERE quantity="volume"';

    $q = $pdo->query($sqltemp);
    $q->setFetchMode(PDO::FETCH_ASSOC);

    while ($row = $q->fetch()) {
        $array_wl[$row['time']] = $row['value'];
    }
} catch (PDOException $e) {
    die("Could not connect to the database $dbname :" . $e->getMessage());
}

$min_level = min($array_wl);
if (!empty($array_wl)) {
    $avg_level = round(array_sum($array_wl) / count($array_wl), 2);
}
else
{
     $avg_level = 0;
}
$client = new Mosquitto\Client();
$client->onConnect('connect');
$client->onDisconnect('disconnect');
$client->onSubscribe('subscribe');
$client->onMessage('message');
$client->setCredentials("pi", "canlab5586");
$client->connect("localhost", 1883, 10);
$client->onLog('logger');
// Create connection

$client->subscribe('uwls/cmd/relay1', 1);
$client->subscribe('uwls/cmd/relay2', 1);
$client->subscribe('uwls/raw/dist', 1);
$client->subscribe('uwls/calc/volume', 1);
$client->subscribe('uwls/calc/tankLevel', 1);

//$client->subscribe('uwls/debug', 1);
for ($i = 0; $i < 10; $i++) {
    $client->loop();
}
//$client->unsubscribe('#', 1);
$client->unsubscribe('uwls/cmd/relay1');
$client->unsubscribe('uwls/cmd/relay2');
$client->unsubscribe('uwls/raw/dist');
$client->unsubscribe('uwls/calc/volume');
$client->unsubscribe('uwls/calc/tankLevel');
//$client->unsubscribe('uwls/debug', 1);
for ($i = 0; $i < 10; $i++) {
    $client->loop();
}
if (isset($_POST["rele1"])) {
    $client->publish('uwls/cmd/relay1', ($relay1 == 1 ? 0 : 1), 1, true);
}
if (isset($_POST["rele2"])) {
    $client->publish('uwls/cmd/relay2', ($relay2 == 1 ? 0 : 1), 1, true);
}
?>
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8"/>
        <meta name="viewport" content="width=device-width, initial-scale=1"/>
        <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css" integrity="sha384-BVYiiSIFeK1dGmJRAkycuHAHRg32OmUcww7on3RYdg4Va+PmSTsz/K68vbdEjh4u" crossorigin="anonymous">
        <link href="//netdna.bootstrapcdn.com/twitter-bootstrap/2.3.2/css/bootstrap-combined.min.css" rel="stylesheet" id="bootstrap-css">
        <script src="//netdna.bootstrapcdn.com/twitter-bootstrap/2.3.2/js/bootstrap.min.js"></script>
        <script src="//code.jquery.com/jquery-1.11.1.min.js"></script>
        <link rel="stylesheet" type="text/css" href="style.css?version=1">
        <style type="text/css">
        input[type="submit"].btn-block, input[type="reset"].btn-block, input[type="button"].btn-block {
        display: inline;
        width: 49%;
        margin-top: 3px;
        }
        .form-group {
        margin-bottom: 0px;
        }
        .form-horizontal .form-group {
        margin-right: 5px;
        margin-left: 10px;
        }
        .row, .thumbnails {
        margin-left: 1;
        }
        .btn-block + .btn-block {
        margin-top: 5px;
        }
        .btn-lg, .btn-group-lg > .btn {
        font-weight: bold;
        padding: 5px 16px;
        font-size: 30px;
        line-height: 1.3333333;
        border-radius: 6px;
        }
        </style>
        <title>ULWS </title>
    </head>
    <body>
        <div class="row">
            <div class="col-lg-1 col-sm-6">
                <div class="circle-tile">
                    <div class="circle-tile-content dark-blue">
                        <div class="circle-tile-description text-faded">
                            Množství vody
                        </div>
                        <div class="circle-tile-number text-faded">
                            <?php echo $volume; ?>
                            <span id="sparklineA"></span>
                        </div>
                    </div>
                </div>
                <div class="circle-tile">
                    <div class="circle-tile-content dark-blue">
                        <div class="circle-tile-description text-faded">
                            Výška hladiny
                        </div>
                        <div class="circle-tile-number text-faded">
                            <?php echo $distance; ?>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-lg-1 col-sm-4">
                <div class="progress vertical">
                    <div class="progress-bar progress-bar-info" role="progressbar" aria-valuenow="100" aria-valuemin="0" aria-valuemax="100" style="width: <?php echo $tankLevel . "%"; ?>">
                    </div>
                </div>
            </div>
            <div class="col-lg-1 intre col-sm-6">
                <div class="circle-tile">
                    <div class="circle-tile-content dark-blue">
                        <div class="circle-tile-description text-faded">
                            Minimální stav
                        </div>
                        <div class="circle-tile-number text-faded">
                            <?php echo $min_level; ?>
                        </div>
                    </div>
                </div>
                <a href="index.php">
                    <div class="circle-tile">
                        <div class="circle-tile-content dark-blue">
                            <div class="circle-tile-description text-faded">
                                Prúměrný stav
                            </div>
                            <div class="circle-tile-number text-faded">
                                <?php echo $avg_level; ?>
                            </div>
                        </div>
                    </div>
                </a>
            </div>

        </div>
         <form class="form-horizontal" role="form" method="post" action="sensor_uwls.php">
                <div class="form-group">
                    <div class="col-md-4">
                        <input id="rele2" name="rele2" type="submit" value="Relé 2" class="btn btn-block btn-lg btn-info">
                        <input id="rele1" name="rele1" type="submit" value="Relé 1" class="btn btn-block btn-lg btn-primary">
                    </div>
                </div>
            </form>
    </body>
</html>