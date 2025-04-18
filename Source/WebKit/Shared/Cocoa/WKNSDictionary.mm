/*
 * Copyright (C) 2013 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#import "config.h"
#import "WKNSDictionary.h"

#import "WKNSArray.h"
#import <WebCore/WebCoreObjCExtras.h>
#import <wtf/AlignedStorage.h>

using namespace WebKit;

@implementation WKNSDictionary {
    AlignedStorage<API::Dictionary> _dictionary;
}

- (Ref<API::Dictionary>)_protectedDictionary
{
    return *_dictionary;
}

- (void)dealloc
{
    if (WebCoreObjCScheduleDeallocateOnMainRunLoop(WKNSDictionary.class, self))
        return;

    self._protectedDictionary->~Dictionary();

    [super dealloc];
}

#pragma mark NSDictionary primitive methods

- (instancetype)initWithObjects:(const id [])objects forKeys:(const id <NSCopying> [])keys count:(NSUInteger)count
{
    ASSERT_NOT_REACHED();
    self = [super initWithObjects:objects forKeys:keys count:count];
    return self;
}

- (NSUInteger)count
{
    return _dictionary->size();
}

- (id)objectForKey:(id)key
{
    RetainPtr str = dynamic_objc_cast<NSString>(key);
    if (!str)
        return nil;

    bool exists;
    RefPtr value = self._protectedDictionary->get(str.get(), exists);
    if (!exists)
        return nil;

    return value ? (id)value->wrapper() : [NSNull null];
}

- (NSEnumerator *)keyEnumerator
{
    return [wrapper(self._protectedDictionary->keys()) objectEnumerator];
}

#pragma mark NSCopying protocol implementation

- (id)copyWithZone:(NSZone *)zone
{
    return [self retain];
}

#pragma mark WKObject protocol implementation

- (API::Object&)_apiObject
{
    return *_dictionary;
}

@end
