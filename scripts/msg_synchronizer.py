import itertools
import threading
from functools import reduce

import rospy
from message_filters import SimpleFilter
from ambf_msgs.msg import RigidBodyState, CameraState
from sensor_msgs.msg import Image


class TimeSynchronizer(SimpleFilter):
    """
    Synchronizes messages by their timestamps.

    :class:`TimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. TimeSynchronizer
    listens on multiple input message filters ``fs``, and invokes the callback
    when it has a collection of messages with matching timestamps.

    The signature of the callback function is::

        def callback(msg1, ... msgN):

    where N is the number of input message filters, and each message is
    the output of the corresponding filter in ``fs``.
    The required ``queue size`` parameter specifies how many sets of
    messages it should store from each input filter (by timestamp)
    while waiting for messages to arrive and complete their "set".
    """

    def __init__(self, fs, queue_size, reset=False):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()
        self.enable_reset = reset

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.latest_stamps = [rospy.Time(0) for f in fs]
        self.input_connections = [
            f.registerCallback(self.add, q, i_q)
            for i_q, (f, q) in enumerate(zip(fs, self.queues))]

    def add(self, msg, my_queue, my_queue_index=None):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]

        ## for debugging
        # if isinstance(msg, Image):
        #     print("received an image")
        #     offset = max(my_queue)
        #     for q in self.queues:
        #         # print(len(q), min(q), max(q))
        #         print(len(q), min(q) - offset, max(q) - offset)

        # common is the set of timestamps that occur in all queues
        common = reduce(set.intersection, [set(q) for q in self.queues])
        for t in sorted(common):
            # msgs is list of msgs (one from each queue) with stamp t
            msgs = [q[t] for q in self.queues]
            self.signalMessage(*msgs)
            for q in self.queues:
                del q[t]
        self.lock.release()
