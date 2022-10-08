#!/usr/bin/python
# This script sends sqs messages to a queue
import boto
from boto.sqs.message import Message

conn = boto.connect_sqs()
q = conn.create_queue('model_results')
m = Message()
m.set_body('Model Run Success')
status = q.write(m)
print status
