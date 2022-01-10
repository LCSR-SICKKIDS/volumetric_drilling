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


class ApproximateTimeSynchronizer(TimeSynchronizer):
    """
    Approximately synchronizes messages by their timestamps.

    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. The API is the same as TimeSynchronizer
    except for an extra `slop` parameter in the constructor that defines the delay (in seconds)
    with which messages can be synchronized. The ``allow_headerless`` option specifies whether
    to allow storing headerless messages with current ROS time instead of timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    """

    def __init__(self, fs, queue_size, slop, allow_headerless=False, reset=False):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = rospy.Duration.from_sec(slop)
        self.allow_headerless = allow_headerless
        self.last_added = rospy.Time()
        self.enable_reset = reset

    def add(self, msg, my_queue, my_queue_index=None):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            if not self.allow_headerless:
                rospy.logwarn("Cannot use message filters with non-stamped messages. "
                              "Use the 'allow_headerless' constructor option to "
                              "auto-assign ROS time to headerless messages.")
                return
            stamp = rospy.Time.now()
        else:
            stamp = msg.header.stamp

        self.lock.acquire()
        now = rospy.Time.now()
        is_simtime = not rospy.rostime.is_wallclock()
        if is_simtime and self.enable_reset and my_queue_index is not None:
            if now < self.latest_stamps[my_queue_index]:
                print("Detected jump back in time. Clearing message filter queue.", msg.name)
                rospy.logdebug("Detected jump back in time. Clearing message filter queue")
                my_queue.clear()
            self.latest_stamps[my_queue_index] = now
        my_queue[stamp] = msg

        # clear all buffers if jump backwards in time is detected
        now = rospy.Time.now()
        if now < self.last_added:
            print("ApproximateTimeSynchronizer: Detected jump back in time. Clearing buffer.", msg.name)
            rospy.loginfo("ApproximateTimeSynchronizer: Detected jump back in time. Clearing buffer.")
            for q in self.queues:
                q.clear()
        self.last_added = now

        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]

        if is_simtime and self.enable_reset:
            if max(self.latest_stamps) != now:
                self.lock.release()
                return
        # self.queues = [topic_0 {stamp: msg}, topic_1 {stamp: msg}, ...]
        if my_queue_index is None:
            search_queues = self.queues
        else:
            search_queues = self.queues[:my_queue_index] + \
                            self.queues[my_queue_index + 1:]
        # sort and leave only reasonable stamps for synchronization
        stamps = []
        for queue in search_queues:
            topic_stamps = []
            for s in queue:
                stamp_delta = abs(s - stamp)
                if stamp_delta > self.slop:
                    continue  # far over the slop
                topic_stamps.append((s, stamp_delta))
            if not topic_stamps:
                print("not topic_stamps", msg.name)
                self.lock.release()
                return
            topic_stamps = sorted(topic_stamps, key=lambda x: x[1])
            stamps.append(topic_stamps)
        for vv in itertools.product(*[next(iter(zip(*s))) for s in stamps]):
            vv = list(vv)
            # insert the new message
            if my_queue_index is not None:
                vv.insert(my_queue_index, stamp)
            qt = list(zip(self.queues, vv))
            if (((max(vv) - min(vv)) < self.slop) and
                    (len([1 for q, t in qt if t not in q]) == 0)):
                msgs = [q[t] for q, t in qt]
                self.signalMessage(*msgs)
                for q, t in qt:
                    del q[t]
                break  # fast finish after the synchronization
        self.lock.release()
