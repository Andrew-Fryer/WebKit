/*
 * Copyright (C) 2024 Apple Inc. All rights reserved.
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

#import <Foundation/Foundation.h>
#import <wtf/Platform.h>

#if ENABLE(WRITING_TOOLS) && PLATFORM(IOS_FAMILY)

#import <UIKit/UIKit.h>

NS_HEADER_AUDIT_BEGIN(nullability, sendability)

typedef NS_ENUM(NSInteger, WKTextAnimationType) {
    WKTextAnimationTypeInitial,
    WKTextAnimationTypeSource,
    WKTextAnimationTypeFinal
};

@protocol WKTextAnimationSourceDelegate;

NS_SWIFT_UI_ACTOR
@interface WKTextAnimationManager : NSObject
- (instancetype)initWithDelegate:(id <WKTextAnimationSourceDelegate>)delegate NS_DESIGNATED_INITIALIZER;
- (void)addTextAnimationForAnimationID:(NSUUID *)uuid withStyleType:(WKTextAnimationType)styleType;
- (void)removeTextAnimationForAnimationID:(NSUUID *)uuid;
@end

NS_SWIFT_UI_ACTOR
@protocol WKTextAnimationSourceDelegate <NSObject>
- (void)targetedPreviewForID:(NSUUID *)uuid completionHandler:(NS_SWIFT_UI_ACTOR void (^)(UITargetedPreview * _Nullable))completionHandler;
- (void)updateUnderlyingTextVisibilityForTextAnimationID:(NSUUID *)uuid visible:(BOOL)visible completionHandler:(NS_SWIFT_UI_ACTOR void (^)(void))completionHandler;
- (UIView *)containingViewForTextAnimationType;
- (void)callCompletionHandlerForAnimationID:(NSUUID *)uuid completionHandler:(NS_SWIFT_UI_ACTOR void (^)(UITargetedPreview * _Nullable))completionHandler;
- (void)callCompletionHandlerForAnimationID:(NSUUID *)uuid;
- (void)replacementEffectDidComplete;
@end

NS_HEADER_AUDIT_END(nullability, sendability)

#endif // ENABLE(WRITING_TOOLS) && PLATFORM(IOS_FAMILY)
