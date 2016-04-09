#!/bin/bash
while true
do
	node app.js
	sleep 5
	echo "Node or the app crashed? Restarting..."
done
