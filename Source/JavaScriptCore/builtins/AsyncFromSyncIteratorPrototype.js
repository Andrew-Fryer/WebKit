/*
 * Copyright (C) 2017 Oleksandr Skachkov <gskachkov@gmail.com>.
 * Copyright (C) 2019 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

@linkTimeConstant
function asyncFromSyncIteratorOnRejected(error, context)
{
    "use strict";

    var syncIterator = context.@syncIterator;
    var promise = context.@promise;

    @assert(@isObject(syncIterator) || syncIterator === @undefined);
    @assert(@isPromise(promise));

    if (syncIterator !== @undefined) {
        var returnMethod;
        try {
            returnMethod = syncIterator.return;
        } catch (e) {
            return @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
        }
        returnMethod.@call(syncIterator);
    }

    return @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, error);
}

@linkTimeConstant
function asyncFromSyncIteratorOnFulfilledContinue(result, context)
{
    "use strict";

    @assert(@isPromise(context.@promise));

    return @resolvePromiseWithFirstResolvingFunctionCallCheck(context.@promise, { value: result, done: false });
}

@linkTimeConstant
function asyncFromSyncIteratorOnFulfilledDone(result, context)
{
    "use strict";

    @assert(@isPromise(context.@promise));

    return @resolvePromiseWithFirstResolvingFunctionCallCheck(context.@promise, { value: result, done: true });
}

function next(value)
{
    "use strict";

    @assert(@isAsyncFromSyncIterator(this));

    var promise = @newPromise();

    if (!@isObject(this) || !@isObject(@getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldSyncIterator))) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator is not an object.'));
        return promise;
    }

    var syncIterator = @getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldSyncIterator);
    var nextMethod = @getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldNextMethod);

    try {
        var nextResult = @argumentCount() === 0 ? nextMethod.@call(syncIterator) : nextMethod.@call(syncIterator, value);
        var onFulfilled = nextResult.done ? @asyncFromSyncIteratorOnFulfilledDone : @asyncFromSyncIteratorOnFulfilledContinue;
        @resolveWithoutPromiseForAsyncAwait(nextResult.value, onFulfilled, @asyncFromSyncIteratorOnRejected, { @promise: promise, @syncIterator: syncIterator });
    } catch (e) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
    }

    return promise;
}

function return(value)
{
    "use strict";

    @assert(@isAsyncFromSyncIterator(this));

    var promise = @newPromise();

    if (!@isObject(this) || !@isObject(@getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldSyncIterator))) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator is not an object.'));
        return promise;
    }

    var syncIterator = @getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldSyncIterator);

    var returnMethod;

    try {
        returnMethod = syncIterator.return;
    } catch (e) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
        return promise;
    }

    if (@isUndefinedOrNull(returnMethod)) {
        @resolvePromiseWithFirstResolvingFunctionCallCheck(promise, { value, done: true });
        return promise;
    }
    
    try {
        var returnResult = @argumentCount() === 0 ? returnMethod.@call(syncIterator) : returnMethod.@call(syncIterator, value);

        if (!@isObject(returnResult)) {
            @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator result interface is not an object.'));
            return promise;
        }

        var onFulfilled = returnResult.done ? @asyncFromSyncIteratorOnFulfilledDone : @asyncFromSyncIteratorOnFulfilledContinue;
        @resolveWithoutPromiseForAsyncAwait(returnResult.value, onFulfilled, @asyncFromSyncIteratorOnRejected, { @promise: promise, @syncIterator: @undefined });
    } catch (e) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
    }

    return promise;
}

function throw(exception)
{
    "use strict";

    @assert(@isAsyncFromSyncIterator(this));

    var promise = @newPromise();

    if (!@isObject(this) || !@isObject(@getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldSyncIterator))) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator is not an object.'));
        return promise;
    }

    var syncIterator = @getAsyncFromSyncIteratorInternalField(this, @asyncFromSyncIteratorFieldSyncIterator);

    var throwMethod;

    try {
        throwMethod = syncIterator.throw;
    } catch (e) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
        return promise;
    }

    if (@isUndefinedOrNull(throwMethod)) {
        var returnMethod;
        try {
            returnMethod = syncIterator.return;
        } catch (e) {
            @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
            return promise;
        }
        var returnResult = returnMethod.@call(syncIterator);
        if (!@isObject(returnResult)) {
            @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator result interface is not an object.'));
            return promise;
        }
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator does not provide a throw method.'));
        return promise;
    }
    
    try {
        var throwResult = @argumentCount() === 0 ? throwMethod.@call(syncIterator) : throwMethod.@call(syncIterator, exception);
        
        if (!@isObject(throwResult)) {
            @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, @makeTypeError('Iterator result interface is not an object.'));
            return promise;
        }
        
        var onFulfilled = throwResult.done ? @asyncFromSyncIteratorOnFulfilledDone : @asyncFromSyncIteratorOnFulfilledContinue;
        @resolveWithoutPromiseForAsyncAwait(throwResult.value, onFulfilled, @asyncFromSyncIteratorOnRejected, { @promise: promise, @syncIterator: syncIterator });
    } catch (e) {
        @rejectPromiseWithFirstResolvingFunctionCallCheck(promise, e);
    }
    
    return promise;
}

@linkTimeConstant
function createAsyncFromSyncIterator(syncIterator, nextMethod)
{
    "use strict";

    if (!@isObject(syncIterator))
        @throwTypeError('Only objects can be wrapped by async-from-sync wrapper');

    return @asyncFromSyncIteratorCreate(syncIterator, nextMethod);
}
