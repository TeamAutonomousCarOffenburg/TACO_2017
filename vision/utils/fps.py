# From http://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
import datetime


class FPS:
    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        # start the timer
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        # stop the timer
        self._end = datetime.datetime.now()
        return self

    def update(self):
        # increment the total number of frames examined during the
        # start and end intervals
        self._numFrames += 1

    def elapsed(self, now=False):
        # return the total number of seconds between the start and
        # end interval
        if now:
            return (datetime.datetime.now() - self._start).total_seconds()
        else:
            return (self._end - self._start).total_seconds()

    def fps(self, now=False):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed(now)
