<?php

$target_dir = "/data/";

$target_file = $target_dir . $_POST["fileName"];

$target_dir = substr($target_file, 0, strlen($target_file) - strlen(basename($target_file)));
if(!is_dir($target_dir)){
    mkdir($target_dir, 0777, true);
}

if (!move_uploaded_file($_FILES["fileToUpload"]["tmp_name"], $target_file)) {
    http_response_code(500);
    exit();
}
