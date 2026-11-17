"""'transparent' numba functionality replacements

njit decorator
data types

dtype_2int_tuple = typeof((1, 1))
@njit(b1(i4, i4, i4[:, :]), cache=True)
@njit(dtype_2int_tuple(f8, f8), cache=True)
"""

from collections.abc import Callable
from typing import Any, TypeVar

F = TypeVar("F", bound=Callable[..., Any])


def njit(*args: Any, **kwargs: Any) -> Callable[[F], F]:
    def wrapper(f: F) -> F:
        return f

    return wrapper


class SubscriptAndCallable:
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        pass

    def __class_getitem__(cls, item: Any) -> type["SubscriptAndCallable"]:
        # an array type, e.g. the return type in ``b1[:](i8[:])``
        return cls


# DTYPES
# @njit(b1(i4, i4, i4[:, :]), cache=True)


class b1(SubscriptAndCallable):
    pass


class f8(SubscriptAndCallable):
    pass


class i2(SubscriptAndCallable):
    pass


class i4(SubscriptAndCallable):
    pass


class i8(SubscriptAndCallable):
    pass


class u2(SubscriptAndCallable):
    pass


class void(SubscriptAndCallable):
    pass


def typeof(*args: Any, **kwargs: Any) -> type[void]:
    return void
