<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);

$temp = "0";
$humidity = "0";
$dewPoint = "0";
$heatIndex = "0";
$distance = "0";

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
    global  $temp,  $humidity, $heatIndex, $dewPoint, $distance ;
    //printf("Got a message on topic %s with payload:\n%s\n", $message->topic, $message->payload);
    if (strcmp($message->topic, "uwls/raw/temp") == 0) 
    {
        $temp = $message->payload;
    } 
    elseif (strcmp($message->topic, "uwls/raw/humid") == 0) 
    {
        $humidity = $message->payload;
    }
    elseif (strcmp($message->topic, "uwls/raw/dist") == 0) 
    {
        $distance = $message->payload;
    }
    elseif (strcmp($message->topic, "uwls/calc/dewPoint") == 0) 
    {
        $dewPoint = $message->payload;
    }
    elseif (strcmp($message->topic, "uwls/calc/heatIndex") == 0) 
    {
        $heatIndex = $message->payload; 
    }  
    elseif (strcmp($message->topic, "uwls/calc/volume") == 0) 
    {
        echo "i equals 2";
    }
    elseif (strcmp($message->topic, "uwls/debug") == 0) 
    {
        echo "i equals 2";
    }  
    echo"";
}
function disconnect()
{
   echo "";
}
function logger()
{
    echo "";//var_dump(func_get_args());
}

$client = new Mosquitto\Client();
$client->onConnect('connect');
$client->onDisconnect('disconnect');
$client->onSubscribe('subscribe');
$client->onMessage('message');
$client->setCredentials("pi", "canlab5586");
$client->connect("localhost", 1883, 10);
$client->onLog('logger');

$client->subscribe('uwls/raw/temp', 1);
$client->subscribe('uwls/raw/humid', 1);
$client->subscribe('uwls/raw/dist', 1);
$client->subscribe('uwls/calc/dewPoint', 1);
$client->subscribe('uwls/calc/heatIndex', 1);
//$client->subscribe('uwls/calc/volume', 1);
//$client->subscribe('uwls/debug', 1);

for ($i = 0; $i < 10; $i++) {
    $client->loop();
}
//$client->unsubscribe('#', 1);
$client->unsubscribe('uwls/raw/temp');
$client->unsubscribe('uwls/raw/humid');
$client->unsubscribe('uwls/raw/dist');
$client->unsubscribe('uwls/calc/dewPoint');
$client->unsubscribe('uwls/calc/heatIndex');
//$client->unsubscribe('uwls/calc/volume', 1);
//$client->unsubscribe('uwls/debug', 1);

for ($i = 0; $i < 10; $i++) {
    $client->loop();
}

?>
<html>
    <head>
        <meta charset="utf-8"/>
        <meta name="viewport" content="width=device-width, initial-scale=1"/>
        <meta http-equiv="refresh" content="30" > 
        <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css" integrity="sha384-BVYiiSIFeK1dGmJRAkycuHAHRg32OmUcww7on3RYdg4Va+PmSTsz/K68vbdEjh4u" crossorigin="anonymous">
        <script src="//code.jquery.com/jquery-1.11.1.min.js"></script>
        <link href="css/bootstrap.css" rel="stylesheet" type="text/css">
        <script src="https://use.fontawesome.com/07b0ce5d10.js"></script>
        <link rel="stylesheet" type="text/css" href="style.css">

        <style type="text/css">
        body{
            background-color: #1a184c;            
        }
        .col-lg-1
        {
         padding-right: 1px;
            padding-left: 15px;   
        }
                .col-sm-6
        {
   width: 20%; 
        }
        </style>
        <script>
        !function(d,s,id){var js,fjs=d.getElementsByTagName(s)[0];if(!d.getElementById(id)){js=d.createElement(s);js.id=id;js.src='https://weatherwidget.io/js/widget.min.js';fjs.parentNode.insertBefore(js,fjs);}}(document,'script','weatherwidget-io-js');
        function autoReload() {
        setTimeout(function() {
        $.ajax({
        url: '/sensor_dht11.php',
        success: function(data) {
        document.getElementById("circle-tile").innerHTML = data;
        }
        });
        autoReload();  // calling again after 5 seconds
        }, 5000);
        }
        autoReload(); // calling the function for the first
        </script>
        <title></title>
    </head>
    <body>
        <div class="container-fluid">
            <div class="row">
                <a class="weatherwidget-io" href="https://forecast7.com/cs/49d2016d61/brno/" data-label_1="BRNO" data-icons="Climacons Animated" data-days="3" data-theme="dark" >BRNO</a>
            </div>
            <div class="row">

                <div class="col-lg-1 col-sm-6">
                    <div class="circle-tile">
                        <div class="circle-tile-content dark-blue">
                            <div class="circle-tile-description text-faded">
                                Naměřená teplota
                            </div>
                            <div class="circle-tile-number text-faded">
                                <?php echo $temp; ?>
                                <span id="sparklineA"></span>
                            </div>
                        </div>
                    </div>
                </div>
                <div class="col-lg-1 col-sm-6">
                    <a href="index.php">
                    <div class="circle-tile">
                        <div class="circle-tile-content dark-gray">
                            <div class="circle-tile-description text-faded">
                                Pocitová teplota
                            </div>
                            <div class="circle-tile-number text-faded">
                                <?php echo $heatIndex; ?>
                            </div>
                        </div>
                    </div>
                </a>
                </div>
                <div class="col-lg-1 col-sm-6">
                    <div class="circle-tile">
                        <div class="circle-tile-content dark-blue">
                            <div class="circle-tile-description text-faded">
                                Naměřená vlhost
                            </div>
                            <div class="circle-tile-number text-faded">
                               <?php echo $humidity; ?>
                            </div>
                        </div>
                    </div>
                </div>
                <div class="col-lg-1 col-sm-6">
                    <div class="circle-tile">
                        <div class="circle-tile-content dark-gray">
                            <div class="circle-tile-description text-faded">
                                Rosný bod
                            </div>
                            <div class="circle-tile-number text-faded">
                              <?php echo $dewPoint; ?> 
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
</body>
</html>