#  Copyright (c) 2024 Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import time


class CacheUtils:
    def __init__(self):
        pass

    @staticmethod
    def expire_by_ttl(seconds=10):
        """A function used to calculate the cache time-to-live hash.

        It works by returning a value that will be the same for a given number of seconds.
        When used as a hash to @functools.lru_cache, it will cause the cache to expire
        after the given number of seconds, triggering a refresh of the cache.

        Note: Since the time is rounded to the nearest second, the first call using this function
        might expire quicker than expected. All subsequent calls should expire at the expected time.
        """
        return round(time.time() / seconds)

    @staticmethod
    def expire_by_value(*args, **kwargs):
        """A function used to calculate the cache key hash.

        It works by returning a value that will be the same for a given set of arguments.
        When used as a hash to @functools.lru_cache, it will cause the cache to be
        invalidated when the arguments change.
        """
        return hash((args, frozenset(kwargs.items())))


def test_cache_ttl():
    # Test that the cache_ttl function returns the same value for a given number of seconds
    hash1 = CacheUtils.expire_by_ttl(1)
    hash2 = CacheUtils.expire_by_ttl(1)
    assert hash1 == hash2, "The cache hash should be the same since it was called immediately after the first call."
    time.sleep(1)
    hash3 = CacheUtils.expire_by_ttl(1)
    assert hash1 != hash3, "The cache hash should have changed after 1 second."
    assert hash2 != hash3, "The cache hash should have changed after 1 second."


def test_cache_key():
    hash1 = CacheUtils.expire_by_value(1, 2, 3, a=4, b=5, c=6)
    hash2 = CacheUtils.expire_by_value(1, 2, 3, a=4, b=5, c=6)
    hash3 = CacheUtils.expire_by_value(1, 2, 3, a=4, b=5, c=7)
    assert hash1 == hash2, "The cache hash should be the same since the arguments are the same."
    assert hash1 != hash3, "The cache hash should be different since the arguments are different."
    assert hash2 != hash3, "The cache hash should be different since the arguments are different."


if __name__ == '__main__':
    test_cache_ttl()
    test_cache_key()
