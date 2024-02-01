from __future__ import annotations
import typing
__all__ = ['createEvent', 'createSemaphore', 'createSignalObject', 'destroyEvent', 'destroySemaphore', 'destroySignalObject', 'releaseSemaphore', 'resetEvent', 'resetSignalObject', 'setEvent', 'setSignalObject', 'waitForObject', 'waitForObjects']
def createEvent(manualReset: bool = False, initialState: bool = False) -> int:
    """
    Creates an event.  Events have binary state (signaled or not signaled) and
    may be either automatically reset or manually reset.  Automatic-reset events
    go to non-signaled state when a WaitForObject is woken up by the event;
    manual-reset events require ResetEvent() to be called to set the event to
    non-signaled state; if ResetEvent() is not called, any waiter on that event
    will immediately wake when called.
    
    :param manualReset:  true for manual reset, false for automatic reset
    :param initialState: true to make the event initially in signaled state
    
    :returns: Event handle
    """
def createSemaphore(initialCount: int = 0, maximumCount: int = 2147483647) -> int:
    """
    Creates a semaphore.  Semaphores keep an internal counter.  Releasing the
    semaphore increases the count.  A semaphore with a non-zero count is
    considered signaled.  When a waiter wakes up it atomically decrements the
    count by 1.  This is generally useful in a single-supplier,
    multiple-consumer scenario.
    
    :param initialCount: initial value for the semaphore's internal counter
    :param maximumCount: maximum value for the samephore's internal counter
    
    :returns: Semaphore handle
    """
def createSignalObject(handle: int, manualReset: bool = False, initialState: bool = False) -> None:
    """
    Sets up signaling for an arbitrary handle.  With this function, any handle
    can operate like an event handle.
    
    :param handle:       Event handle
    :param manualReset:  true for manual reset, false for automatic reset
    :param initialState: true to make the handle initially in signaled state
    """
def destroyEvent(handle: int) -> None:
    """
    Destroys an event.  Destruction wakes up any waiters.
    
    :param handle: event handle
    """
def destroySemaphore(handle: int) -> None:
    """
    Destroys a semaphore.  Destruction wakes up any waiters.
    
    :param handle: semaphore handle
    """
def destroySignalObject(handle: int) -> None:
    """
    Cleans up signaling for a handle.  Destruction wakes up any waiters.
    
    :param handle: handle
    """
def releaseSemaphore(handle: int, releaseCount: int = 1) -> tuple[bool, int]:
    """
    Releases N counts of a semaphore.
    
    :param handle:       semaphore handle
    :param releaseCount: amount to add to semaphore's internal counter;
                         must be positive
    :param prevCount:    if non-null, previous count (output parameter)
    
    :returns: True on successful release, false on failure (e.g. release count
              would exceed maximum value, or handle invalid)
    """
def resetEvent(handle: int) -> None:
    """
    Sets an event to non-signaled state.
    
    :param handle: event handle
    """
def resetSignalObject(handle: int) -> None:
    """
    Sets a handle to non-signaled state.
    
    :param handle: handle
    """
def setEvent(handle: int) -> None:
    """
    Sets an event to signaled state.
    
    :param handle: event handle
    """
def setSignalObject(handle: int) -> None:
    """
    Sets a handle to signaled state.
    
    :param handle: handle
    """
@typing.overload
def waitForObject(handle: int) -> bool:
    """
    Waits for an handle to be signaled.
    
    :param handle: handle to wait on
    
    :returns: True if handle was signaled, false otherwise (e.g. object was
              destroyed)
    """
@typing.overload
def waitForObject(handle: int, timeout: float) -> tuple[bool, bool]:
    """
    Waits for an handle to be signaled, with timeout.
    
    :param handle:   handle to wait on
    :param timeout:  timeout in seconds
    :param timedOut: if non-null, set to true if timeout reached without handle
                     being signaled; set to false otherwise (output)
    
    :returns: True if handle was signaled, false otherwise (e.g. object was
              destroyed or timed out)
    """
@typing.overload
def waitForObjects(handles: list[int]) -> list[int]:
    """
    Waits for one or more handles to be signaled.
    
    Invalid handles are treated as signaled; the returned array will have the
    handle error bit set for any invalid handles.
    
    :param handles:  array of handles to wait on
    :param signaled: output array for storage of signaled handles; must be at
                     least the size of the handles input array
    
    :returns: array of signaled handles (points into signaled array)
    """
@typing.overload
def waitForObjects(handles: list[int], timeout: float) -> tuple[list[int], bool]:
    """
    Waits for one or more handles to be signaled, with timeout.
    
    Invalid handles are treated as signaled; the returned array will have the
    handle error bit set for any invalid handles.
    
    :param handles:  array of handles to wait on
    :param signaled: output array for storage of signaled handles; must be at
                     least the size of the handles input array
    :param timeout:  timeout in seconds
    :param timedOut: if non-null, set to true if timeout reached without any
                     handle being signaled; set to false otherwise (output)
    
    :returns: array of signaled handles (points into signaled array)
    """
