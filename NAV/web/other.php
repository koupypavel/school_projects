<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);
$width= "";
$height ="";
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
global $distance;
//printf("Got a message on topic %s with payload:\n%s\n", $message->topic, $message->payload);
if (strcmp($message->topic, "uwls/raw/dist") == 0) {
$distance = $message->payload;
} elseif (strcmp($message->topic, "uwls/calc/volume") == 0) {
echo "i equals 2";
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
if (isset($_POST["submit"])) {
$width  = $_POST['width'];
$height = $_POST['height'];
$client = new Mosquitto\Client();
$client->onConnect('connect');
$client->onDisconnect('disconnect');
$client->setCredentials("pi", "canlab5586");
$client->connect("localhost", 1883, 10);
$client->onLog('logger');
$client->publish('uwls/cmd/width', "1" . $width, 1, true);
$client->publish('uwls/cmd/height', "1" . $height, 1, true);
echo "";
}
if (isset($_POST["calibration"])) {
$client = new Mosquitto\Client();
$client->onConnect('connect');
$client->onDisconnect('disconnect');
$client->setCredentials("pi", "canlab5586");
$client->connect("localhost", 1883, 10);
$client->onLog('logger');
$client->publish('uwls/cmd/calibration', "1", 1, true);
}
if (isset($_POST["keyboard"])) {
exec('xterm -iconic -e "matchbox-keyboard"');
}
?>
<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="utf-8">
        <meta http-equiv="X-UA-Compatible" content="IE=edge">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.1/css/bootstrap.min.css">
        <link rel="stylesheet" type="text/css" href="style.css?version=1">
        <style>
        .error {color: #FF0000;}
        h3, form
        {
        color: yellowgreen;
        }
        .form-horizontal .control-label
        {
        padding-top: 0;
        text-align: left;
        float:left;
        width: 160px;
        text-align: left;
        }
        .btn-lg, .btn-group-lg > .btn
        {
        font-weight: normal;
        padding: 5px 16px;
        font-size: 20px;
        line-height: 1.3333333;
        border-radius: 6px;
        }
        .container {
        margin-top: 0px;
        padding-top: 0px;
        /*background-color: #beb*/
        }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="row">
                <div class="col-md-6 col-md-offset-3">
                    <h3>Nastavení</h3>
                    <form class="form-horizontal" role="form" method="post" action="setting.php">
                        <div class="form-group">
                            <label for="width" class="col-sm-2 control-label">Výška nádrže</label>
                            <div class="col-sm-10">
                                <input type="height" class="form-control" id="height" name="height" placeholder="xx cm" value="<?php echo htmlspecialchars($height); ?>">
                            </div>
                        </div>
                        <div class="form-group">
                            <label for="width" class="col-sm-2 control-label">Průměr/šířka nádrže</label>
                            <div class="col-sm-10">
                                <input type="width" class="form-control" id="width" name="width" placeholder="xx cm" value="<?php echo htmlspecialchars($width); ?>">
                            </div>
                        </div>
                        <div class="form-group">
                            <div class="col-sm-10 col-sm-offset-2">
                                <input id="submit" name="submit" type="submit" value="Uložit" class="btn btn-primary">
                                <input id="calibration" name="calibration" type="submit" value="Kalibrace" class="btn btn-info">
                                <input id="keyboard" name="keyboard" type="submit" value="Klávesnice" class="btn btn-success">
                                <a href="index.php">
                                    <button type="button" class="btn btn-danger">
                                    Zpět
                                    </button>
                                </a>
                            </div>
                        </div>
                    </form>
                </div>
            </div>
        </div>
        <script src="https://ajax.googleapis.com/ajax/libs/jquery/1.11.0/jquery.min.js"></script>
        <script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.1/js/bootstrap.min.js"></script>
    </body>
</html>