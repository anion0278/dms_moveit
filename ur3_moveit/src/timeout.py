import signal
from contextlib import contextmanager
import time


@contextmanager
def __timeout(time, timeout_handler):
    signal.signal(signal.SIGALRM, signal.SIG_IGN)

    # Register a function to raise a TimeoutError on the signal.
    signal.signal(signal.SIGALRM, __raise_timeout)
    # Schedule the signal to be sent after ``time``.
    signal.alarm(time)

    try:
        yield
    except RuntimeError:
        if timeout_handler is None:
            raise RuntimeError
        else:
            timeout_handler
    finally:
        # Unregister the signal so it won't be triggered
        # if the timeout is not reached.
        signal.signal(signal.SIGALRM, signal.SIG_IGN)


def __raise_timeout(signum, frame):
    print("Timeout")
    raise RuntimeError


def action_with_timeout(action, time, timeout_handler=None):
    # Add a timeout block.
    with __timeout(time, timeout_handler):
        print('Starting action with timeout...')
        action()
