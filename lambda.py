from __future__ import print_function
import json
import logging
import os
import boto3
import time
from time import gmtime, strftime
from botocore.exceptions import ClientError

logger = logging.getLogger()
logger.setLevel(logging.INFO)
logger.info("My Lambda")


def build_speechlet_response(title, output, reprompt_text, should_end_session):
    return {
        'outputSpeech': {
            'type': 'PlainText',
            'text': output
        },
        'card': {
            'type': 'Simple',
            'title': title,
            'content': output
        },
        'reprompt': {
            'outputSpeech': {
                'type': 'PlainText',
                'text': reprompt_text
            }
        },
        'shouldEndSession': should_end_session
    }

def build_response(session_attributes, speechlet_response):
    return {
        'version': '1.0',
        'sessionAttributes': session_attributes,
        'response': speechlet_response
    }


# --------------- Functions that control the skill's behavior ------------------
def get_test_response(intent):
    session_attributes = {}
    card_title = "Drone"
    slot1 = intent['slots']['direction']['value']
    slot2 = intent['slots']['distance']['value']
    flag = False
    table = boto3.resource('dynamodb').Table('status')
    try:
        # scan_response = table.scan(ProjectionExpression='present_status')
        response = table.get_item(
            Key={
                'time': 'now',
            }
        )
        present_status = response['Item'].get('present_status')
        logger.info("Status found")
    except ClientError:
        logger.exception("couldn't find status")
        status_code = 404
    if slot1:
        speech_output = "Your Drone pilot has issued a command to move the drone towards "+ slot1 +" for "+ slot2 +" meters. To know the status ask report status or land or return to launch."
        flag = False
    else:
        flag = True
        speech_output = "the given is an unintended direction"
    #----------------Store into log files-------------------------
    dynamodb = boto3.resource('dynamodb')
    table = dynamodb.Table('DroneLogFiles')
    now = strftime("%d %b %Y %H:%M:%S +0000", gmtime())
    response = table.put_item(
    Item={
        'id': now,
        'direction':slot1,
        'distance': slot2,
        })
    message = (slot1.upper())+slot2
    #-----------------------------------------------------------------
    #------------Post message to client ------------------------------
    table = boto3.resource('dynamodb').Table('table_name')
    connection_ids = []
    try:
        scan_response = table.scan(ProjectionExpression='connection_id')
        connection_ids = [item['connection_id'] for item in scan_response['Items']]
        logger.info("Found %s active connections.", len(connection_ids))
    except ClientError:
        logger.exception("Couldn't get connections.")
        status_code = 404
    for i in connection_ids:
        apig_management_client = boto3.client('apigatewaymanagementapi', endpoint_url=f'https://*****************')

        send_response = apig_management_client.post_to_connection(Data=message, ConnectionId=i)
    reprompt_text = "I didn't get what you were trying to say. Please repeat the command again."
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

#----------------Getting the status of drone from the dynamodb-------------------- 
def get_status_response(intent):
    session_attributes = {}
    card_title = "Drone"
    table = boto3.resource('dynamodb').Table('status')
    try:
        response = table.get_item(
            Key={
                'time': 'now',
            }
        )
        altitude = response['Item'].get('altidude')
        location = response['Item'].get('location')
        logger.info("Status found")
    except ClientError:
        logger.exception("couldn't find status")
        status_code = 404
    speech_output = "Drone is flying in loiter mode at an altidude of" + altitude + "at" + location
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))
    
# this is a takeoff code
def get_takeoff():
    session_attributes = {}
    card_title = "Takeoff"
    speech_output = "Ok. Initiating take off. To know the status, say where is my drone?"
    message ='TAKEOFF'
    table = boto3.resource('dynamodb').Table('table_name')
    connection_ids = []
    try:
        scan_response = table.scan(ProjectionExpression='connection_id')
        connection_ids = [item['connection_id'] for item in scan_response['Items']]
        logger.info("Found %s active connections.", len(connection_ids))
    except ClientError:
        logger.exception("Couldn't get connections.")
        status_code = 404
    for i in connection_ids:
        apig_management_client = boto3.client('apigatewaymanagementapi', endpoint_url=f'https://*****************')
        send_response = apig_management_client.post_to_connection(Data=message, ConnectionId=i)
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))
        
def get_rtl():
    session_attributes = {}
    card_title = "Landing"
    message ='RTL'
    table = boto3.resource('dynamodb').Table('table_name')
    connection_ids = []
    try:
        scan_response = table.scan(ProjectionExpression='connection_id')
        connection_ids = [item['connection_id'] for item in scan_response['Items']]
        logger.info("Found %s active connections.", len(connection_ids))
    except ClientError:
        logger.exception("Couldn't get connections.")
        status_code = 404
    for i in connection_ids:
        apig_management_client = boto3.client('apigatewaymanagementapi', endpoint_url=f'https://*****************')
        send_response = apig_management_client.post_to_connection(Data=message, ConnectionId=i)
    should_end_session = False
    speech_output = "Ok RTL command has been issued. Say report status to check the status."
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))

def get_whereismydrone():
    session_attributes = {}
    card_title = "Welcome"
    table = boto3.resource('dynamodb').Table('status')
    try:
        # scan_response = table.scan(ProjectionExpression='present_status')
        response = table.get_item(
            Key={
                'time': 'now',
            }
        )
        location = response['Item'].get('location')
        altitude = response['Item'].get('altitude')
        if altitude==0:
            speech_output = "Your drone is at " + location + "location. To move the drone, say ask drone pilot to move north by 5 meters"
        else:
            speech_output = "Your drone is flying at an altitude of " + altidude +" at " + + location + ". To move the drone, say ask drone pilot to move north by 5 meters"
    except ClientError:
        logger.exception("couldn't find location or altitude")
        status_code = 404
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(card_title, speech_output, None, should_end_session))

def get_welcome_response():
    """ If we wanted to initialize the session to have some attributes we could
    add those here
    """
    session_attributes = {}
    card_title = "Welcome"
    table = boto3.resource('dynamodb').Table('status')
    try:
        # scan_response = table.scan(ProjectionExpression='present_status')
        response = table.get_item(
            Key={
                'time': 'now',
            }
        )
        present_status = response['Item'].get('present_status')
        if present_status=='online':
            speech_output = 'Your drone pilot is online. Shall we begin?'
        else:
            speech_output = 'Your drone pilot is offline. Please try again later'
    except ClientError:
        logger.exception("couldn't find status")
        status_code = 404
   
    # If the user either does not reply to the welcome message or says something
    # that is not understood, they will be prompted again with this text.
    reprompt_text = "I don't know if you heard me, welcome to your custom alexa application!"
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))


def handle_session_end_request():
    card_title = "Disconnected"
    table = boto3.resource('dynamodb').Table('table_name')
    connection_ids = []
    try:
        scan_response = table.scan(ProjectionExpression='connection_id')
        connection_ids = [item['connection_id'] for item in scan_response['Items']]
        logger.info("Found %s active connections.", len(connection_ids))
    except ClientError:
        logger.exception("Couldn't get connections.")
        status_code = 404
    for i in connection_ids:
        apig_management_client = boto3.client('apigatewaymanagementapi', endpoint_url=f'https://*****************')
        send_response = apig_management_client.post_to_connection(Data=message, ConnectionId=i)
    should_end_session = False
    speech_output = "Ok RTL command has been issued. Say report status to check the status."
    should_end_session = True
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))

# --------------- Events ------------------

def on_session_started(session_started_request, session):
    """ Called when the session starts.
        One possible use of this function is to initialize specific 
        variables from a previous state stored in an external database
    """
    # Add additional code here as needed
    pass

    

def on_launch(launch_request, session):
    """ Called when the user launches the skill without specifying what they
    want
    """
    # Dispatch to your skill's launch message
    return get_welcome_response()

def get_wrong_response():
    session_attributes = {}
    card_title = "Wrong"
    speech_output = "Sorry, I did not understand. Try saying where is my drone."
    # If the user either does not reply to the welcome message or says something
    # that is not understood, they will be prompted again with this text.
    reprompt_text = "give valid command to your custom alexa application!"
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

def on_intent(intent_request, session):
    """ Called when the user specifies an intent for this skill """

    intent = intent_request['intent']
    intent_name = intent_request['intent']['name']

    # Dispatch to your skill's intent handlers
    if intent_name == "movement":
        return get_test_response(intent)
    elif intent_name == "getdronestatus":
        return get_status_response(intent)
    elif intent_name == 'takeoff':
        return get_takeoff()
    elif intent_name == "rtl":
        return get_rtl()
    elif intent_name == "whereismydrone":
        return get_whereismydrone()
    elif intent_name == 'AMAZON.YESIntent':
        return get_yes_intent()
    elif intent_name == "AMAZON.HelpIntent":
        return get_welcome_response()
    elif intent_name == "AMAZON.FallbackIntent":
        return get_wrong_response()
    elif intent_name == "AMAZON.CancelIntent" or intent_name == "AMAZON.StopIntent":
        return handle_session_end_request()
    else:
        raise ValueError("Invalid intent")


def on_session_ended(session_ended_request, session):
    """ Called when the user ends the session.

    Is not called when the skill returns should_end_session=true
    """
    print("on_session_ended requestId=" + session_ended_request['requestId'] +
          ", sessionId=" + session['sessionId'])
    # add cleanup logic here


# --------------- Main handler ------------------

def lambda_handler(event, context):
    """ Route the incoming request based on type (LaunchRequest, IntentRequest,
    etc.) The JSON body of the request is provided in the event parameter.
    """
    print("Incoming request...")

    """
    Uncomment this if statement and populate with your skill's application ID to
    prevent someone else from configuring a skill that sends requests to this
    function.
    """
    # if (event['session']['application']['applicationId'] !=
    #         "amzn1.echo-sdk-ams.app.[unique-value-here]"):
    #     raise ValueError("Invalid Application ID")

    if event['session']['new']:
        on_session_started({'requestId': event['request']['requestId']},
                           event['session'])

    if event['request']['type'] == "LaunchRequest":
        return on_launch(event['request'], event['session'])
    elif event['request']['type'] == "IntentRequest":
        return on_intent(event['request'], event['session'])
    elif event['request']['type'] == "SessionEndedRequest":
        return on_session_ended(event['request'], event['session'])

# -----------------------------------------------------------------------------------

def handle_connect(user_name, table, connection_id):
    """
    Handles new connections by adding the connection ID and user name to the
    DynamoDB table.

    :param user_name: The name of the user that started the connection.
    :param table: The DynamoDB connection table.
    :param connection_id: The websocket connection ID of the new connection.
    :return: An HTTP status code that indicates the result of adding the connection
             to the DynamoDB table.
    """
    status_code = 200
    try:
        table.put_item(
            Item={'connection_id': connection_id, 'user_name': user_name})
        logger.info(
            "Added connection %s for user %s.", connection_id, user_name)
    except ClientError:
        logger.exception(
            "Couldn't add connection %s for user %s.", connection_id, user_name)
        status_code = 503
    return status_code


def handle_disconnect(table, connection_id):
    """
    Handles disconnections by removing the connection record from the DynamoDB table.

    :param table: The DynamoDB connection table.
    :param connection_id: The websocket connection ID of the connection to remove.
    :return: An HTTP status code that indicates the result of removing the connection
             from the DynamoDB table.
    """
    status_code = 200
    try:
        table.delete_item(Key={'connection_id': connection_id})
        logger.info("Disconnected connection %s.", connection_id)
    except ClientError:
        logger.exception("Couldn't disconnect connection %s.", connection_id)
        status_code = 503
    return status_code


def handle_message(table, connection_id, event_body, apig_management_client):
    """
    Handles messages sent by a participant in the chat. Looks up all connections
    currently tracked in the DynamoDB table, and uses the API Gateway Management API
    to post the message to each other connection.

    When posting to a connection results in a GoneException, the connection is
    considered disconnected and is removed from the table. This is necessary
    because disconnect messages are not always sent when a client disconnects.

    :param table: The DynamoDB connection table.
    :param connection_id: The ID of the connection that sent the message.
    :param event_body: The body of the message sent from API Gateway. This is a
                       dict with a `msg` field that contains the message to send.
    :param apig_management_client: A Boto3 API Gateway Management API client.
    :return: An HTTP status code that indicates the result of posting the message
             to all active connections.
    """
    status_code = 200
    user_name = 'guest'
    try:
        item_response = table.get_item(Key={'connection_id': connection_id})
        user_name = item_response['Item']['user_name']
        logger.info("Got user name %s.", user_name)
    except ClientError:
        logger.exception("Couldn't find user name. Using %s.", user_name)

    connection_ids = []
    try:
        scan_response = table.scan(ProjectionExpression='connection_id')
        connection_ids = [item['connection_id'] for item in scan_response['Items']]
        logger.info("Found %s active connections.", len(connection_ids))
    except ClientError:
        logger.exception("Couldn't get connections.")
        status_code = 404

    message = f"{user_name}: {event_body['message']}".encode('utf-8')
    logger.info("Message: %s", message)

    for other_conn_id in connection_ids:
        try:
            if other_conn_id != connection_id:
                if event_body['sender']=='gs':
                    dynamodb = boto3.resource('dynamodb')
                    table = dynamodb.Table('status')
                    response = table.put_item(
                    Item={
                        'time': "now",
                        'present_status' : event_body['message'],
                        'longitude' : event_body['longitude'],
                        'latitude' : event_body['latitude'],
                        'altitude' : event_body['altitude'],
                    })
                else:
                    send_response = apig_management_client.post_to_connection(
                        Data=message, ConnectionId=other_conn_id)
                    logger.info(
                        "Posted message to connection %s, got response %s.",
                        other_conn_id, send_response)
        except ClientError:
            logger.exception("Couldn't post to connection %s.", other_conn_id)
        except apig_management_client.exceptions.GoneException:
            logger.info("Connection %s is gone, removing.", other_conn_id)
            try:
                table.delete_item(Key={'connection_id': other_conn_id})
            except ClientError:
                logger.exception("Couldn't remove connection %s.", other_conn_id)

    return status_code


def lambda_handler_ws(event, context):
    # print(event)
    if 'requestContext' not in event.keys():
        return lambda_handler(event,context)
    # print(event['requestContext']['identity']['userAgent'])
    """
    An AWS Lambda handler that receives events from an API Gateway websocket API
    and dispatches them to various handler functions.

    This function looks up the name of a DynamoDB table in the `table_name` environment
    variable. The table must have a primary key named `connection_id`.

    This function handles three routes: $connect, $disconnect, and sendmessage. Any
    other route results in a 404 status code.

    The $connect route accepts a query string `name` parameter that is the name of
    the user that originated the connection. This name is added to all chat messages
    sent by that user.

    :param event: A dict that contains request data, query string parameters, and
                  other data sent by API Gateway.
    :param context: Context around the request.
    :return: A response dict that contains an HTTP status code that indicates the
             result of handling the event.
    """
    table_name = 'table_name' #os.environ['table_name']
    route_key = event.get('requestContext', {}).get('routeKey')
    connection_id = event.get('requestContext', {}).get('connectionId')
    if table_name is None or route_key is None or connection_id is None:
        return {'statusCode': 400}

    table = boto3.resource('dynamodb').Table(table_name)
    logger.info("Request: %s, use table %s.", route_key, table.name)

    response = {'statusCode': 200}
    logger.info(route_key)
    if route_key == '$connect':
        user_name = event.get('queryStringParameters', {'name': 'guest'}).get('name')
        response['statusCode'] = handle_connect(user_name, table, connection_id)
    elif route_key == '$disconnect':
        response['statusCode'] = handle_disconnect(table, connection_id)
    elif route_key == 'sendmessage':
        body = event.get('body')
        body = json.loads(body if body is not None else '{"msg": ""}')
        domain = event.get('requestContext', {}).get('domainName')
        stage = event.get('requestContext', {}).get('stage')
        if domain is None or stage is None:
            logger.warning(
                "Couldn't send message. Bad endpoint in request: domain '%s', "
                "stage '%s'", domain, stage)
            response['statusCode'] = 400
        else:
            apig_management_client = boto3.client(
                'apigatewaymanagementapi', endpoint_url=f'https://{domain}/{stage}')
            # if event_body['sender']=='gs':
            #         t = dynamodb.Table('status')
            #         resp = t.put_item(
            #         Item={
            #             'time': "now",
            #             'present_status' : bod['message'],
            #         })

            response['statusCode'] = handle_message(table, connection_id, body, apig_management_client)
    else:
        response['statusCode'] = 404

    return response
