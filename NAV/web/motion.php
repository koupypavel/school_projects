<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);

$host     = "localhost";
$username = "nav";
$password = "polda2541";
$dbname   = "hexiwear";

$accelerometr = "x:0.05 y:0.12 z:0.08";
$gyroscope  = "x:0.1 y:0.0 z:0.5";
$magnetometr  = "x:0.00 y:0.01 z:0.00";
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
    global $accelerometr, $magnetometr, $gyroscope;
//printf("Got a message on topic %s with payload:\n%s\n", $message->topic, $message->payload);
    if (strcmp($message->topic, "hexiwear/2001/motion_service/accelerometr") == 0) {
        $accelerometr = $message->payload;
    } elseif (strcmp($message->topic, "hexiwear/2002/motion_service/gyroscope") == 0) {
        $gyroscope = $message->payload;
    }
    elseif (strcmp($message->topic, "hexiwear/2002/motion_service/magnetometr") == 0) {
        $magnetometr = $message->payload;
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

$client = new Mosquitto\Client();
$client->onConnect('connect');
$client->onDisconnect('disconnect');
$client->onSubscribe('subscribe');
$client->onMessage('message');
$client->connect("localhost", 1883, 10);
$client->onLog('logger');
// Create connection
/*
"hexiwear/2001/motion_service/accelerometr";
constexpr auto MQTT_HEXIWEAR_MOTION_SERVICE_GYRO    = "hexiwear/2002/motion_service/gyroscope";
constexpr auto MQTT_HEXIWEAR_MOTION_SERVICE_MAG     = "hexiwear/2003/motion_service/magnetometr"
*/
$client->subscribe('hexiwear/2001/motion_service/accelerometr', 0);
$client->subscribe('hexiwear/2002/motion_service/gyroscope', 0);
$client->subscribe('hexiwear/2003/motion_service/magnetometr', 0);

//$client->subscribe('uwls/debug', 1);
for ($i = 0; $i < 10; $i++) {
    $client->loop();
}
//$client->unsubscribe('#', 1);
$client->unsubscribe('hexiwear/2001/motion_service/accelerometr');
$client->unsubscribe('hexiwear/2002/motion_service/gyroscope');
$client->unsubscribe('hexiwear/2003/motion_service/magnetometr');

for ($i = 0; $i < 10; $i++) {
    $client->loop();
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
                            gyroscope
                        </div>
                        <div class="circle-tile-number text-faded">
                            <?php echo $gyroscope; ?>
                            <span id="sparklineA"></span>
                        </div>
                    </div>
                </div>
                <div class="circle-tile">
                    <div class="circle-tile-content dark-blue">
                        <div class="circle-tile-description text-faded">
                            accelerometr
                        </div>
                        <div class="circle-tile-number text-faded">
                            <?php echo $accelerometr; ?>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-lg-1 intre col-sm-6">
                <a href="index.php">
                    <div class="circle-tile">
                        <div class="circle-tile-content dark-blue">
                            <div class="circle-tile-description text-faded">
                                magnetometr
                            </div>
                            <div class="circle-tile-number text-faded">
                                <?php echo $magnetometr; ?>
                            </div>
                        </div>
                    </div>
                </a>
            </div>

        </div>
         <form class="form-horizontal" role="form" method="post" action="motion.php">
                <div class="form-group">
                    <div class="col-md-4">
                    </div>
                </div>
            </form>
    </body>
</html>