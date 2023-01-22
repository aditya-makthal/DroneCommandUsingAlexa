# Drone Command and Control using Alexa

*The main logic of how alexa handles intent is written lambda.py an AWS Lambda code.
*The request to drone is sent through websockets to the ground station.
*The ground station then using private network sends the command to the drone using the code written in GroundStationCodes
