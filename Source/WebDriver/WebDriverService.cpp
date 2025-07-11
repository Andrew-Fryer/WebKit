/*
 * Copyright (C) 2017 Igalia S.L.
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

#include "config.h"
#include "WebDriverService.h"

#include "Capabilities.h"
#include "CommandResult.h"
#include "Logging.h"
#include "SessionHost.h"
#include <cmath>
#include <ranges>
#include <wtf/Compiler.h>
#include <wtf/LoggerHelper.h>
#include <wtf/RunLoop.h>
#include <wtf/SortedArrayMap.h>
#include <wtf/text/MakeString.h>
#include <wtf/text/StringToIntegerConversion.h>
#include <wtf/text/WTFString.h>

#if ENABLE(WEBDRIVER_BIDI)
#include "HTTPServer.h"
#include "WebSocketServer.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <limits>
#include <optional>
#include <wtf/JSONValues.h>
#include <wtf/StdLibExtras.h>
#include <wtf/UUID.h>
#include <wtf/glib/GTypedefs.h>
#endif

namespace WebDriver {

// https://w3c.github.io/webdriver/webdriver-spec.html#dfn-maximum-safe-integer
static const double maxSafeInteger = 9007199254740991.0; // 2 ^ 53 - 1

WebDriverService::WebDriverService()
    : m_server(*this)
#if ENABLE(WEBDRIVER_BIDI)
    , m_bidiServer(WebSocketServer::create(*this))
    , m_browserTerminatedObserver([this](const String& sessionID) { onBrowserTerminated(sessionID); })
#endif
{
#if ENABLE(WEBDRIVER_BIDI)
    SessionHost::addBrowserTerminatedObserver(m_browserTerminatedObserver);
#endif
}

WebDriverService::~WebDriverService()
{
#if ENABLE(WEBDRIVER_BIDI)
    SessionHost::removeBrowserTerminatedObserver(m_browserTerminatedObserver);
#endif
}

static void printUsageStatement(const char* programName)
{
    SAFE_PRINTF("Usage: %s options\n", String::fromLatin1(programName).utf8());
    SAFE_PRINTF("  -h,          --help             Prints this help message\n");
    SAFE_PRINTF("  -p <port>,   --port=<port>      Port number the driver will use\n");
    SAFE_PRINTF("               --host=<host>      Host IP the driver will use, or either 'local' or 'all' (default: 'local')\n");
    SAFE_PRINTF("  -t <ip:port> --target=<ip:port> Target IP and port\n");
#if ENABLE(WEBDRIVER_BIDI)
    SAFE_PRINTF("               --bidi-port=<port>        Port number to use for BiDi's WebSocket connections\n");
#endif
    SAFE_PRINTF("               --replace-on-new-session  Replace the existing session on new session request\n");
}

int WebDriverService::run(int argc, char** argv)
{
    String portString;
    std::optional<String> host;
#if ENABLE(WEBDRIVER_BIDI)
    String bidiPortString;
#endif
    String targetString;
    if (const char* targetEnvVar = getenv("WEBDRIVER_TARGET_ADDR"))
        targetString = String::fromLatin1(targetEnvVar);
    for (int i = 1 ; i < argc; ++i) {
        auto arg = unsafeSpan(argv[i]);
        if (equalSpans(arg, "-h"_span) || equalSpans(arg, "--help"_span)) {
            printUsageStatement(argv[0]);
            return EXIT_SUCCESS;
        }

        if (equalSpans(arg, "-p"_span) && portString.isNull()) {
            if (++i == argc) {
                printUsageStatement(argv[0]);
                return EXIT_FAILURE;
            }
            portString = String::fromLatin1(argv[i]);
            continue;
        }

        static constexpr auto portArgument = "--port="_span;
        if (spanHasPrefix(arg, portArgument) && portString.isNull()) {
            portString = arg.subspan(portArgument.size());
            continue;
        }

        static constexpr auto hostArgument = "--host="_span;
        if (spanHasPrefix(arg, hostArgument) && !host) {
            host = arg.subspan(hostArgument.size());
            continue;
        }

#if ENABLE(WEBDRIVER_BIDI)
        static constexpr auto bidiPortArgument = "--bidi-port="_span;
        if (spanHasPrefix(arg, bidiPortArgument) && bidiPortString.isNull()) {
            bidiPortString = arg.subspan(bidiPortArgument.size());
            continue;
        }
#endif

        if (equalSpans(arg, "-t"_span) && targetString.isNull()) {
            if (++i == argc) {
                printUsageStatement(argv[0]);
                return EXIT_FAILURE;
            }
            targetString = String::fromLatin1(argv[i]);
            continue;
        }

        static constexpr auto targetArgument = "--target="_span;
        if (spanHasPrefix(arg, targetArgument) && targetString.isNull()) {
            targetString = arg.subspan(targetArgument.size());
            continue;
        }

        if (equalSpans(arg, "--replace-on-new-session"_span)) {
            m_replaceOnNewSession = true;
            continue;
        }
    }

    if (portString.isNull()) {
        printUsageStatement(argv[0]);
        return EXIT_FAILURE;
    }

    if (!targetString.isEmpty()) {
        auto position = targetString.reverseFind(':');
        if (position != notFound) {
            m_targetAddress = targetString.left(position);
            m_targetPort = parseIntegerAllowingTrailingJunk<uint16_t>(StringView { targetString }.substring(position + 1)).value_or(0);
        }
    }

    auto port = parseInteger<uint16_t>(portString);
    if (!port) {
        fprintf(stderr, "Invalid port %s provided\n", portString.utf8().data());
        return EXIT_FAILURE;
    }

#if ENABLE(WEBDRIVER_BIDI)
    auto bidiPort = parseInteger<uint16_t>(bidiPortString);
    if (!bidiPort) {
        const int16_t bidiPortIncrement = *port == std::numeric_limits<uint16_t>::max() ? -1 : 1;
        bidiPort = { *port + bidiPortIncrement };
        fprintf(stderr, "Invalid WebSocket BiDi port %s provided. Defaulting to %d.\n", bidiPortString.utf8().data(), *bidiPort);
    }
#endif

    WTF::initializeMainThread();

    const char* hostStr = host && host->utf8().data() ? host->utf8().data() : "local";
#if ENABLE(WEBDRIVER_BIDI)
    if (!m_bidiServer->listen(host ? *host : nullString(), *bidiPort)) {
        fprintf(stderr, "FATAL: Unable to listen for WebSocket BiDi server at host %s and port %d.\n", hostStr, *bidiPort);
        return EXIT_FAILURE;
    }
    RELEASE_LOG(WebDriverBiDi, "Started WebSocket BiDi server with host %s and port %d", hostStr, *bidiPort);
#endif // ENABLE(WEBDRIVER_BIDI)
    if (!m_server.listen(host, *port)) {
        fprintf(stderr, "FATAL: Unable to listen for HTTP server at host %s and port %d.\n", hostStr, *port);
        return EXIT_FAILURE;
    }
    RELEASE_LOG(WebDriverClassic, "Started HTTP server with host %s and port %d", hostStr, *port);

    RunLoop::run();

#if ENABLE(WEBDRIVER_BIDI)
    m_bidiServer->disconnect();
#endif
    m_server.disconnect();

    return EXIT_SUCCESS;
}

const WebDriverService::Command WebDriverService::s_commands[] = {
    { HTTPMethod::Post, "/session", &WebDriverService::newSession },
    { HTTPMethod::Delete, "/session/$sessionId", &WebDriverService::deleteSession },
    { HTTPMethod::Get, "/status", &WebDriverService::status },
    { HTTPMethod::Get, "/session/$sessionId/timeouts", &WebDriverService::getTimeouts },
    { HTTPMethod::Post, "/session/$sessionId/timeouts", &WebDriverService::setTimeouts },

    { HTTPMethod::Post, "/session/$sessionId/url", &WebDriverService::go },
    { HTTPMethod::Get, "/session/$sessionId/url", &WebDriverService::getCurrentURL },
    { HTTPMethod::Post, "/session/$sessionId/back", &WebDriverService::back },
    { HTTPMethod::Post, "/session/$sessionId/forward", &WebDriverService::forward },
    { HTTPMethod::Post, "/session/$sessionId/refresh", &WebDriverService::refresh },
    { HTTPMethod::Get, "/session/$sessionId/title", &WebDriverService::getTitle },

    { HTTPMethod::Get, "/session/$sessionId/window", &WebDriverService::getWindowHandle },
    { HTTPMethod::Delete, "/session/$sessionId/window", &WebDriverService::closeWindow },
    { HTTPMethod::Post, "/session/$sessionId/window", &WebDriverService::switchToWindow },
    { HTTPMethod::Get, "/session/$sessionId/window/handles", &WebDriverService::getWindowHandles },
    { HTTPMethod::Post, "/session/$sessionId/window/new", &WebDriverService::newWindow },
    { HTTPMethod::Post, "/session/$sessionId/frame", &WebDriverService::switchToFrame },
    { HTTPMethod::Post, "/session/$sessionId/frame/parent", &WebDriverService::switchToParentFrame },
    { HTTPMethod::Get, "/session/$sessionId/window/rect", &WebDriverService::getWindowRect },
    { HTTPMethod::Post, "/session/$sessionId/window/rect", &WebDriverService::setWindowRect },
    { HTTPMethod::Post, "/session/$sessionId/window/maximize", &WebDriverService::maximizeWindow },
    { HTTPMethod::Post, "/session/$sessionId/window/minimize", &WebDriverService::minimizeWindow },
    { HTTPMethod::Post, "/session/$sessionId/window/fullscreen", &WebDriverService::fullscreenWindow },

    { HTTPMethod::Post, "/session/$sessionId/element", &WebDriverService::findElement },
    { HTTPMethod::Post, "/session/$sessionId/elements", &WebDriverService::findElements },
    { HTTPMethod::Post, "/session/$sessionId/element/$elementId/element", &WebDriverService::findElementFromElement },
    { HTTPMethod::Post, "/session/$sessionId/element/$elementId/elements", &WebDriverService::findElementsFromElement },
    { HTTPMethod::Post, "/session/$sessionId/shadow/$shadowId/element", &WebDriverService::findElementFromShadowRoot },
    { HTTPMethod::Post, "/session/$sessionId/shadow/$shadowId/elements", &WebDriverService::findElementsFromShadowRoot },
    { HTTPMethod::Get, "/session/$sessionId/element/active", &WebDriverService::getActiveElement },

    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/shadow", &WebDriverService::getElementShadowRoot },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/selected", &WebDriverService::isElementSelected },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/attribute/$name", &WebDriverService::getElementAttribute },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/property/$name", &WebDriverService::getElementProperty },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/css/$name", &WebDriverService::getElementCSSValue },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/text", &WebDriverService::getElementText },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/name", &WebDriverService::getElementTagName },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/rect", &WebDriverService::getElementRect },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/enabled", &WebDriverService::isElementEnabled },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/computedrole", &WebDriverService::getComputedRole },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/computedlabel", &WebDriverService::getComputedLabel },

    { HTTPMethod::Post, "/session/$sessionId/element/$elementId/click", &WebDriverService::elementClick },
    { HTTPMethod::Post, "/session/$sessionId/element/$elementId/clear", &WebDriverService::elementClear },
    { HTTPMethod::Post, "/session/$sessionId/element/$elementId/value", &WebDriverService::elementSendKeys },

    { HTTPMethod::Get, "/session/$sessionId/source", &WebDriverService::getPageSource },
    { HTTPMethod::Post, "/session/$sessionId/execute/sync", &WebDriverService::executeScript },
    { HTTPMethod::Post, "/session/$sessionId/execute/async", &WebDriverService::executeAsyncScript },

    { HTTPMethod::Get, "/session/$sessionId/cookie", &WebDriverService::getAllCookies },
    { HTTPMethod::Get, "/session/$sessionId/cookie/$name", &WebDriverService::getNamedCookie },
    { HTTPMethod::Post, "/session/$sessionId/cookie", &WebDriverService::addCookie },
    { HTTPMethod::Delete, "/session/$sessionId/cookie/$name", &WebDriverService::deleteCookie },
    { HTTPMethod::Delete, "/session/$sessionId/cookie", &WebDriverService::deleteAllCookies },

    { HTTPMethod::Post, "/session/$sessionId/actions", &WebDriverService::performActions },
    { HTTPMethod::Delete, "/session/$sessionId/actions", &WebDriverService::releaseActions },

    { HTTPMethod::Post, "/session/$sessionId/alert/dismiss", &WebDriverService::dismissAlert },
    { HTTPMethod::Post, "/session/$sessionId/alert/accept", &WebDriverService::acceptAlert },
    { HTTPMethod::Get, "/session/$sessionId/alert/text", &WebDriverService::getAlertText },
    { HTTPMethod::Post, "/session/$sessionId/alert/text", &WebDriverService::sendAlertText },

    { HTTPMethod::Get, "/session/$sessionId/screenshot", &WebDriverService::takeScreenshot },
    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/screenshot", &WebDriverService::takeElementScreenshot },


    { HTTPMethod::Get, "/session/$sessionId/element/$elementId/displayed", &WebDriverService::isElementDisplayed },
};

#if ENABLE(WEBDRIVER_BIDI)
const WebDriverService::BidiCommand WebDriverService::s_bidiCommands[] = {
    { "session.status"_s, &WebDriverService::bidiSessionStatus },
    { "session.subscribe"_s, &WebDriverService::bidiSessionSubscribe },
    { "session.unsubscribe"_s, &WebDriverService::bidiSessionUnsubscribe },
};
#endif

std::optional<WebDriverService::HTTPMethod> WebDriverService::toCommandHTTPMethod(const String& method)
{
    static constexpr std::pair<ComparableLettersLiteral, WebDriverService::HTTPMethod> httpMethodMappings[] = {
        { "delete"_s, WebDriverService::HTTPMethod::Delete },
        { "get"_s, WebDriverService::HTTPMethod::Get },
        { "post"_s, WebDriverService::HTTPMethod::Post },
        { "put"_s, WebDriverService::HTTPMethod::Post },
    };
    static constexpr SortedArrayMap httpMethods { httpMethodMappings };

    if (auto* methodValue = httpMethods.tryGet(method))
        return *methodValue;
    return std::nullopt;
}

bool WebDriverService::findCommand(HTTPMethod method, const String& path, CommandHandler* handler, HashMap<String, String>& parameters)
{
    size_t length = std::size(s_commands);
    for (size_t i = 0; i < length; ++i) {
        if (s_commands[i].method != method)
            continue;

        Vector<String> pathTokens = path.split('/');
        Vector<String> commandTokens = String::fromUTF8(s_commands[i].uriTemplate).split('/');
        if (pathTokens.size() != commandTokens.size())
            continue;

        bool allMatched = true;
        for (size_t j = 0; j < pathTokens.size() && allMatched; ++j) {
            if (commandTokens[j][0] == '$')
                parameters.set(commandTokens[j].substring(1), pathTokens[j]);
            else if (commandTokens[j] != pathTokens[j])
                allMatched = false;
        }

        if (allMatched) {
            *handler = s_commands[i].handler;
            return true;
        }

        parameters.clear();
    }

    return false;
}

void WebDriverService::handleRequest(HTTPRequestHandler::Request&& request, Function<void (HTTPRequestHandler::Response&&)>&& replyHandler)
{
    auto method = toCommandHTTPMethod(request.method);
    if (!method) {
        sendResponse(WTFMove(replyHandler), CommandResult::fail(CommandResult::ErrorCode::UnknownCommand, makeString("Unknown method: "_s, request.method)));
        return;
    }
    CommandHandler handler;
    HashMap<String, String> parameters;
    if (!findCommand(method.value(), request.path, &handler, parameters)) {
        sendResponse(WTFMove(replyHandler), CommandResult::fail(CommandResult::ErrorCode::UnknownCommand, makeString("Unknown command: "_s, request.path)));
        return;
    }

    RefPtr<JSON::Object> parametersObject;
    if (method.value() == HTTPMethod::Post) {
        auto messageValue = JSON::Value::parseJSON(String::fromUTF8({ request.data, request.dataLength }));
        if (!messageValue) {
            sendResponse(WTFMove(replyHandler), CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid JSON in request body"_s));
            return;
        }

        parametersObject = messageValue->asObject();
        if (!parametersObject) {
            sendResponse(WTFMove(replyHandler), CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Expected JSON object in request body"_s));
            return;
        }
    } else
        parametersObject = JSON::Object::create();
    for (const auto& parameter : parameters)
        parametersObject->setString(parameter.key, parameter.value);

    ((*this).*handler)(WTFMove(parametersObject), [this, replyHandler = WTFMove(replyHandler)](CommandResult&& result) mutable {
        sendResponse(WTFMove(replyHandler), WTFMove(result));
    });
}

void WebDriverService::sendResponse(Function<void (HTTPRequestHandler::Response&&)>&& replyHandler, CommandResult&& result) const
{
    // §6.3 Processing Model.
    // https://w3c.github.io/webdriver/webdriver-spec.html#processing-model
    RefPtr<JSON::Value> resultValue;
    if (result.isError()) {
        // When required to send an error.
        // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-send-an-error
        // Let body be a new JSON Object initialised with the following properties: "error", "message", "stacktrace".
        auto errorObject = JSON::Object::create();
        errorObject->setString("error"_s, result.errorString());
        errorObject->setString("message"_s, result.errorMessage().value_or(emptyString()));
        errorObject->setString("stacktrace"_s, emptyString());
        // If the error data dictionary contains any entries, set the "data" field on body to a new JSON Object populated with the dictionary.
        if (auto& additionalData = result.additionalErrorData())
            errorObject->setObject("data"_s, *additionalData);
        // Send a response with status and body as arguments.
        resultValue = WTFMove(errorObject);
    } else if (auto value = result.result())
        resultValue = WTFMove(value);
    else
        resultValue = JSON::Value::null();

    // When required to send a response.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-send-a-response
    auto responseObject = JSON::Object::create();
    responseObject->setValue("value"_s, resultValue.releaseNonNull());
    replyHandler({ result.httpStatusCode(), responseObject->toJSONString().utf8(), "application/json; charset=utf-8"_s });
}

#if ENABLE(WEBDRIVER_BIDI)

bool WebDriverService::acceptHandshake(HTTPRequestHandler::Request&& request)
{
    // https://w3c.github.io/webdriver-bidi/#transport
    auto& resourceName = request.path;

    auto& resources = m_bidiServer->listener()->resources;
    auto foundResource = std::ranges::find(resources, resourceName);
    if (foundResource == resources.end()) {
        RELEASE_LOG(WebDriverBiDi, "Resource name %s not found in listener's list of WebSocket resources. Rejecting handshake.", resourceName.utf8().data());
        return false;
    }

    if (*foundResource == "/session"_s) {
        // FIXME Add support for bidi-only sessions
        RELEASE_LOG(WebDriverBiDi, "BiDi-only sessions are not supported yet. Rejecting handshake.");
        return false;
    }

    auto sessionID = m_bidiServer->getSessionID(resourceName);
    if (sessionID.isNull()) {
        RELEASE_LOG(WebDriverBiDi, "No session ID found for resource name %s. Rejecting handshake.", resourceName.utf8().data());
        return false;
    }

    // FIXME Properly support multiple sessions in the future
    if (sessionID != m_session->id()) {
        RELEASE_LOG(WebDriverBiDi, "No active session found for session ID %s. Rejecting handshake.", sessionID.utf8().data());
        return false;
    }

    return true;
}

void WebDriverService::handleMessage(WebSocketMessageHandler::Message&& message, Function<void(WebSocketMessageHandler::Message&&)>&& completionHandler)
{
    // https://w3c.github.io/webdriver-bidi/#handle-an-incoming-message

    if (!message.connection) {
        RELEASE_LOG(WebDriverBiDi, "Incoming message without attached connection. Ignoring message.");
        completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::UnknownError, std::nullopt));
        return;
    }

    auto connection = message.connection;
    if (m_bidiServer->sessionID(connection) != m_session->id()) {
        // FIXME Remove once we support checking static vs non-static methods https://bugs.webkit.org/show_bug.cgi?id=281721
        completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidSessionID, connection));
        return;
    }

    auto parsedMessageValue = JSON::Value::parseJSON(String::fromUTF8(message.payload.data()));
    if (!parsedMessageValue) {
        RELEASE_LOG(WebDriverBiDi, "WebDriverService::handleMessage() Failed to parse incoming message");
        completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, message.connection));
        return;
    }

    const auto& messageObject = parsedMessageValue->asObject();
    if (!messageObject) {
        RELEASE_LOG_ERROR(WebDriverBiDi, "WebDriver handle BiDi message: Expected object.");
        completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, connection));
        return;
    }

    std::optional<int> commandId = messageObject->getInteger("id"_s);
    if (!commandId) {
        RELEASE_LOG_ERROR(WebDriverBiDi, "Missing command ID.");
        completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, connection, "Missing command id"_s));
        return;
    }

    BidiCommandHandler handler;
    RefPtr<JSON::Object> parameters;
    // FIXME Maybe replace manual method dispatch with generated dispatchers for static methods, like we do in WebDriverBidiProcessor-related classes in the UIProcess
    // https://bugs.webkit.org/show_bug.cgi?id=281721
    if (!findBidiCommand(messageObject, &handler, parameters)) {
        RELEASE_LOG(WebDriverBiDi, "Failed to find appropriate BiDi command on WebDriver service. Relaying to the browser.");
        auto sessionID = m_session->id();
        m_session->relayBidiCommand(makeString(message.payload), *commandId, [completionHandler = WTFMove(completionHandler), sessionID, this](WebSocketMessageHandler::Message&& resultMessage) {
            auto connection = m_bidiServer->connection(sessionID);
            if (!connection) {
                RELEASE_LOG(WebDriverBiDi, "Failed to find connection for session ID %s. Ignoring message.", sessionID.utf8().data());
                return;
            }
            resultMessage.connection = *connection;
            completionHandler(WTFMove(resultMessage));
        });
        return;
    }

    ((*this).*handler)(*commandId, WTFMove(parameters), [completionHandler = WTFMove(completionHandler), message](WebSocketMessageHandler::Message&& resultMessage) {
        // 6.7.5 If method is "session.new", let session be the entry in the list of active sessions whose session ID is equal to the "sessionId" property of value, append connection to session’s session WebSocket connections, and remove connection from the WebSocket connections not associated with a session.
        // FIXME https://bugs.webkit.org/show_bug.cgi?id=281722
        resultMessage.connection = message.connection;
        completionHandler(WTFMove(resultMessage));
    });
}

bool WebDriverService::findBidiCommand(const RefPtr<JSON::Object>& parameters, BidiCommandHandler* handler, RefPtr<JSON::Object>& parsedParams)
{
    const String& method = parameters->getString("method"_s);
    if (!method)
        return false;

    auto candidate = std::find_if(std::begin(s_bidiCommands), std::end(s_bidiCommands),
        [method](const BidiCommand& command) {
            return method == command.method;
    });

    if (candidate == std::end(s_bidiCommands))
        return false;

    parsedParams = parameters->getObject("params"_s);
    *handler = candidate->handler;
    return true;
}

#endif // ENABLE(WEBDRIVER_BIDI)

static std::optional<double> valueAsNumberInRange(const JSON::Value& value, double minAllowed = 0, double maxAllowed = std::numeric_limits<int>::max())
{
    auto number = value.asDouble();
    if (!number)
        return std::nullopt;

    if (std::isnan(*number) || std::isinf(*number))
        return std::nullopt;

    if (*number < minAllowed || *number > maxAllowed)
        return std::nullopt;

    return *number;
}

static std::optional<uint64_t> unsignedValue(JSON::Value& value)
{
    auto number = valueAsNumberInRange(value, 0, maxSafeInteger);
    if (!number)
        return std::nullopt;

    auto intValue = static_cast<uint64_t>(number.value());
    // If the contained value is a double, bail in case it doesn't match the integer
    // value, i.e. if the double value was not originally in integer form.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-integer
    if (number.value() != intValue)
        return std::nullopt;

    return intValue;
}

enum class IgnoreUnknownTimeout : bool { No, Yes };

static std::optional<Timeouts> deserializeTimeouts(JSON::Object& timeoutsObject, IgnoreUnknownTimeout ignoreUnknownTimeout)
{
    // §8.5 Set Timeouts.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-deserialize-as-a-timeout
    Timeouts timeouts;
    auto end = timeoutsObject.end();
    for (auto it = timeoutsObject.begin(); it != end; ++it) {
        if (it->key == "sessionId"_s)
            continue;

        if (it->key == "script"_s && it->value->isNull()) {
            timeouts.script = std::numeric_limits<double>::infinity();
            continue;
        }

        // If value is not an integer, or it is less than 0 or greater than the maximum safe integer, return error with error code invalid argument.
        auto timeoutMS = unsignedValue(it->value);
        if (!timeoutMS)
            return std::nullopt;

        if (it->key == "script"_s)
            timeouts.script = timeoutMS.value();
        else if (it->key == "pageLoad"_s)
            timeouts.pageLoad = timeoutMS.value();
        else if (it->key == "implicit"_s)
            timeouts.implicit = timeoutMS.value();
        else if (ignoreUnknownTimeout == IgnoreUnknownTimeout::No)
            return std::nullopt;
    }
    return timeouts;
}

static std::optional<Proxy> deserializeProxy(JSON::Object& proxyObject)
{
    // §7.1 Proxy.
    // https://w3c.github.io/webdriver/#proxy
    Proxy proxy;

    proxy.type = proxyObject.getString("proxyType"_s);
    if (!proxy.type)
        return std::nullopt;

    if (proxy.type == "direct"_s || proxy.type == "autodetect"_s || proxy.type == "system"_s)
        return proxy;

    if (proxy.type == "pac"_s) {
        auto autoconfigURL = proxyObject.getString("proxyAutoconfigUrl"_s);
        if (!autoconfigURL)
            return std::nullopt;

        proxy.autoconfigURL = URL({ }, autoconfigURL);
        if (!proxy.autoconfigURL->isValid())
            return std::nullopt;

        return proxy;
    }

    if (proxy.type == "manual"_s) {
        if (auto value = proxyObject.getValue("ftpProxy"_s)) {
            auto ftpProxy = value->asString();
            if (!ftpProxy)
                return std::nullopt;

            proxy.ftpURL = URL({ }, makeString("ftp://"_s, ftpProxy));
            if (!proxy.ftpURL->isValid())
                return std::nullopt;
        }
        if (auto value = proxyObject.getValue("httpProxy"_s)) {
            auto httpProxy = value->asString();
            if (!httpProxy)
                return std::nullopt;

            proxy.httpURL = URL({ }, makeString("http://"_s, httpProxy));
            if (!proxy.httpURL->isValid())
                return std::nullopt;
        }
        if (auto value = proxyObject.getValue("sslProxy"_s)) {
            auto sslProxy = value->asString();
            if (!sslProxy)
                return std::nullopt;

            proxy.httpsURL = URL({ }, makeString("https://"_s, sslProxy));
            if (!proxy.httpsURL->isValid())
                return std::nullopt;
        }
        if (auto value = proxyObject.getValue("socksProxy"_s)) {
            auto socksProxy = value->asString();
            if (!socksProxy)
                return std::nullopt;

            proxy.socksURL = URL({ }, makeString("socks://"_s, socksProxy));
            if (!proxy.socksURL->isValid())
                return std::nullopt;

            auto socksVersionValue = proxyObject.getValue("socksVersion"_s);
            if (!socksVersionValue)
                return std::nullopt;

            auto socksVersion = unsignedValue(*socksVersionValue);
            if (!socksVersion || socksVersion.value() > 255)
                return std::nullopt;
            proxy.socksVersion = socksVersion.value();
        }
        if (auto value = proxyObject.getValue("noProxy"_s)) {
            auto noProxy = value->asArray();
            if (!noProxy)
                return std::nullopt;

            auto noProxyLength = noProxy->length();
            for (unsigned i = 0; i < noProxyLength; ++i) {
                auto address = noProxy->get(i)->asString();
                if (!address)
                    return std::nullopt;
                proxy.ignoreAddressList.append(address);
            }
        }

        return proxy;
    }

    return std::nullopt;
}

static std::optional<PageLoadStrategy> deserializePageLoadStrategy(const String& pageLoadStrategy)
{
    if (pageLoadStrategy == "none"_s)
        return PageLoadStrategy::None;
    if (pageLoadStrategy == "normal"_s)
        return PageLoadStrategy::Normal;
    if (pageLoadStrategy == "eager"_s)
        return PageLoadStrategy::Eager;
    return std::nullopt;
}

static std::optional<UnhandledPromptBehavior> deserializeUnhandledPromptBehavior(const String& unhandledPromptBehavior)
{
    if (unhandledPromptBehavior == "dismiss"_s)
        return UnhandledPromptBehavior::Dismiss;
    if (unhandledPromptBehavior == "accept"_s)
        return UnhandledPromptBehavior::Accept;
    if (unhandledPromptBehavior == "dismiss and notify"_s)
        return UnhandledPromptBehavior::DismissAndNotify;
    if (unhandledPromptBehavior == "accept and notify"_s)
        return UnhandledPromptBehavior::AcceptAndNotify;
    if (unhandledPromptBehavior == "ignore"_s)
        return UnhandledPromptBehavior::Ignore;
    return std::nullopt;
}

void WebDriverService::parseCapabilities(const JSON::Object& matchedCapabilities, Capabilities& capabilities) const
{
    // Matched capabilities have already been validated.
    auto acceptInsecureCerts = matchedCapabilities.getBoolean("acceptInsecureCerts"_s);
    if (acceptInsecureCerts)
        capabilities.acceptInsecureCerts = *acceptInsecureCerts;

    auto setWindowRect = matchedCapabilities.getBoolean("setWindowRect"_s);
    if (setWindowRect)
        capabilities.setWindowRect = *setWindowRect;

    auto browserName = matchedCapabilities.getString("browserName"_s);
    if (!!browserName)
        capabilities.browserName = browserName;

    auto browserVersion = matchedCapabilities.getString("browserVersion"_s);
    if (!!browserVersion)
        capabilities.browserVersion = browserVersion;

    auto platformName = matchedCapabilities.getString("platformName"_s);
    if (!!platformName)
        capabilities.platformName = platformName;

    auto proxy = matchedCapabilities.getObject("proxy"_s);
    if (proxy)
        capabilities.proxy = deserializeProxy(*proxy);

    auto strictFileInteractability = matchedCapabilities.getBoolean("strictFileInteractability"_s);
    if (strictFileInteractability)
        capabilities.strictFileInteractability = *strictFileInteractability;

    auto timeouts = matchedCapabilities.getObject("timeouts"_s);
    if (timeouts)
        capabilities.timeouts = deserializeTimeouts(*timeouts, IgnoreUnknownTimeout::No);

    auto pageLoadStrategy = matchedCapabilities.getString("pageLoadStrategy"_s);
    if (!!pageLoadStrategy)
        capabilities.pageLoadStrategy = deserializePageLoadStrategy(pageLoadStrategy);

    auto unhandledPromptBehavior = matchedCapabilities.getString("unhandledPromptBehavior"_s);
    if (!!unhandledPromptBehavior)
        capabilities.unhandledPromptBehavior = deserializeUnhandledPromptBehavior(unhandledPromptBehavior);

    if (auto webSocketURL = matchedCapabilities.getBoolean("webSocketUrl"_s))
        capabilities.webSocketURL = *webSocketURL;

    platformParseCapabilities(matchedCapabilities, capabilities);
}

bool WebDriverService::findSessionOrCompleteWithError(JSON::Object& parameters, Function<void (CommandResult&&)>& completionHandler)
{
    auto sessionID = parameters.getString("sessionId"_s);
    if (!sessionID) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing session ID parameter"_s));
        return false;
    }

    if (!m_session || m_session->id() != sessionID) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidSessionID));
        return false;
    }

    if (!m_session->isConnected()) {
        m_session = nullptr;
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidSessionID, String("session deleted because of page crash or hang."_s)));
        return false;
    }

    return true;
}

RefPtr<JSON::Object> WebDriverService::validatedCapabilities(const JSON::Object& capabilities) const
{
    // §7.2 Processing Capabilities.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-validate-capabilities
    auto result = JSON::Object::create();
    auto end = capabilities.end();
    for (auto it = capabilities.begin(); it != end; ++it) {
        if (it->value->isNull())
            continue;
        if (it->key == "acceptInsecureCerts"_s) {
            auto acceptInsecureCerts = it->value->asBoolean();
            if (!acceptInsecureCerts)
                return nullptr;
            result->setBoolean(it->key, *acceptInsecureCerts);
        } else if (it->key == "browserName"_s || it->key == "browserVersion"_s || it->key == "platformName"_s) {
            auto stringValue = it->value->asString();
            if (!stringValue)
                return nullptr;
            result->setString(it->key, stringValue);
        } else if (it->key == "pageLoadStrategy"_s) {
            auto pageLoadStrategy = it->value->asString();
            if (!pageLoadStrategy || !deserializePageLoadStrategy(pageLoadStrategy))
                return nullptr;
            result->setString(it->key, pageLoadStrategy);
        } else if (it->key == "proxy"_s) {
            auto proxy = it->value->asObject();
            if (!proxy || !deserializeProxy(*proxy))
                return nullptr;
            result->setValue(it->key, *proxy);
        } else if (it->key == "strictFileInteractability"_s) {
            auto strictFileInteractability = it->value->asBoolean();
            if (!strictFileInteractability)
                return nullptr;
            result->setBoolean(it->key, *strictFileInteractability);
        } else if (it->key == "timeouts"_s) {
            auto timeouts = it->value->asObject();
            if (!timeouts || !deserializeTimeouts(*timeouts, IgnoreUnknownTimeout::No))
                return nullptr;
            result->setValue(it->key, *timeouts);
        } else if (it->key == "unhandledPromptBehavior"_s) {
            auto unhandledPromptBehavior = it->value->asString();
            if (!unhandledPromptBehavior || !deserializeUnhandledPromptBehavior(unhandledPromptBehavior))
                return nullptr;
            result->setString(it->key, unhandledPromptBehavior);
        } else if (it->key.find(':') != notFound) {
            if (!platformValidateCapability(it->key, it->value))
                return nullptr;
            result->setValue(it->key, it->value.copyRef());
        } else if (it->key == "webSocketUrl"_s) {
            auto webSocketURL = it->value->asBoolean();
            if (!webSocketURL)
                return nullptr;
            result->setBoolean(it->key, *webSocketURL);
        } else
            return nullptr;
    }
    return result;
}

RefPtr<JSON::Object> WebDriverService::mergeCapabilities(const JSON::Object& requiredCapabilities, const JSON::Object& firstMatchCapabilities) const
{
    // §7.2 Processing Capabilities.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-merging-capabilities
    auto result = JSON::Object::create();
    auto requiredEnd = requiredCapabilities.end();
    for (auto it = requiredCapabilities.begin(); it != requiredEnd; ++it)
        result->setValue(it->key, it->value.copyRef());

    auto firstMatchEnd = firstMatchCapabilities.end();
    for (auto it = firstMatchCapabilities.begin(); it != firstMatchEnd; ++it)
        result->setValue(it->key, it->value.copyRef());

    return result;
}

RefPtr<JSON::Object> WebDriverService::matchCapabilities(const JSON::Object& mergedCapabilities) const
{
    // §7.2 Processing Capabilities.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-matching-capabilities
    Capabilities platformCapabilities = this->platformCapabilities();

    // Some capabilities like browser name and version might need to launch the browser,
    // so we only reject the known capabilities that don't match.
    auto matchedCapabilities = JSON::Object::create();
    if (platformCapabilities.browserName)
        matchedCapabilities->setString("browserName"_s, platformCapabilities.browserName.value());
    if (platformCapabilities.browserVersion)
        matchedCapabilities->setString("browserVersion"_s, platformCapabilities.browserVersion.value());
    if (platformCapabilities.platformName)
        matchedCapabilities->setString("platformName"_s, platformCapabilities.platformName.value());
    if (platformCapabilities.acceptInsecureCerts)
        matchedCapabilities->setBoolean("acceptInsecureCerts"_s, platformCapabilities.acceptInsecureCerts.value());
    if (platformCapabilities.strictFileInteractability)
        matchedCapabilities->setBoolean("strictFileInteractability"_s, platformCapabilities.strictFileInteractability.value());
    if (platformCapabilities.setWindowRect)
        matchedCapabilities->setBoolean("setWindowRect"_s, platformCapabilities.setWindowRect.value());

    auto end = mergedCapabilities.end();
    for (auto it = mergedCapabilities.begin(); it != end; ++it) {
        if (it->key == "browserName"_s && platformCapabilities.browserName) {
            auto browserName = it->value->asString();
            if (!equalIgnoringASCIICase(platformCapabilities.browserName.value(), browserName))
                return nullptr;
        } else if (it->key == "browserVersion"_s && platformCapabilities.browserVersion) {
            auto browserVersion = it->value->asString();
            if (!platformCompareBrowserVersions(browserVersion, platformCapabilities.browserVersion.value()))
                return nullptr;
        } else if (it->key == "platformName"_s && platformCapabilities.platformName) {
            auto platformName = it->value->asString();
            if (!equalLettersIgnoringASCIICase(platformName, "any"_s) && platformCapabilities.platformName.value() != platformName)
                return nullptr;
        } else if (it->key == "acceptInsecureCerts"_s && platformCapabilities.acceptInsecureCerts) {
            auto acceptInsecureCerts = it->value->asBoolean();
            if (acceptInsecureCerts && !platformCapabilities.acceptInsecureCerts.value())
                return nullptr;
        } else if (it->key == "proxy"_s) {
            auto proxyType = it->value->asObject()->getString("proxyType"_s);
            if (!platformSupportProxyType(proxyType))
                return nullptr;
        } else if (it->key == "webSocketUrl"_s) {
            auto webSocketURL = it->value->asBoolean();
            if (webSocketURL && !platformSupportBidi())
                return nullptr;
        } else if (!platformMatchCapability(it->key, it->value))
            return nullptr;
        matchedCapabilities->setValue(it->key, it->value.copyRef());
    }

    return matchedCapabilities;
}

Vector<Capabilities> WebDriverService::processCapabilities(const JSON::Object& parameters, Function<void (CommandResult&&)>& completionHandler) const
{
    // §7.2 Processing Capabilities.
    // https://w3c.github.io/webdriver/webdriver-spec.html#processing-capabilities

    // 1. Let capabilities request be the result of getting the property "capabilities" from parameters.
    auto capabilitiesObject = parameters.getObject("capabilities"_s);
    if (!capabilitiesObject) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "capabilities parameter is missing in request"_s));
        return { };
    }

    // 2. Let required capabilities be the result of getting the property "alwaysMatch" from capabilities request.
    RefPtr<JSON::Object> requiredCapabilities;
    auto requiredCapabilitiesValue = capabilitiesObject->getValue("alwaysMatch"_s);
    if (!requiredCapabilitiesValue) {
        // 2.1. If required capabilities is undefined, set the value to an empty JSON Object.
        requiredCapabilities = JSON::Object::create();
    } else if (!(requiredCapabilities = requiredCapabilitiesValue->asObject())) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, String("alwaysMatch is invalid in capabilities"_s)));
        return { };
    }

    // 2.2. Let required capabilities be the result of trying to validate capabilities with argument required capabilities.
    requiredCapabilities = validatedCapabilities(*requiredCapabilities);
    if (!requiredCapabilities) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, String("Invalid alwaysMatch capabilities"_s)));
        return { };
    }

    // 3. Let all first match capabilities be the result of getting the property "firstMatch" from capabilities request.
    RefPtr<JSON::Array> firstMatchCapabilitiesList;
    auto firstMatchCapabilitiesValue = capabilitiesObject->getValue("firstMatch"_s);
    if (!firstMatchCapabilitiesValue) {
        // 3.1. If all first match capabilities is undefined, set the value to a JSON List with a single entry of an empty JSON Object.
        firstMatchCapabilitiesList = JSON::Array::create();
        firstMatchCapabilitiesList->pushObject(JSON::Object::create());
    } else {
        firstMatchCapabilitiesList = firstMatchCapabilitiesValue->asArray();
        if (!firstMatchCapabilitiesList) {
            // 3.2. If all first match capabilities is not a JSON List, return error with error code invalid argument.
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, String("firstMatch is invalid in capabilities"_s)));
            return { };
        }
    }

    // 4. Let validated first match capabilities be an empty JSON List.
    Vector<RefPtr<JSON::Object>> validatedFirstMatchCapabilitiesList;
    auto firstMatchCapabilitiesListLength = firstMatchCapabilitiesList->length();
    validatedFirstMatchCapabilitiesList.reserveInitialCapacity(firstMatchCapabilitiesListLength);
    // 5. For each first match capabilities corresponding to an indexed property in all first match capabilities.
    for (unsigned i = 0; i < firstMatchCapabilitiesListLength; ++i) {
        auto firstMatchCapabilities = firstMatchCapabilitiesList->get(i)->asObject();
        if (!firstMatchCapabilities) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, String("Invalid capabilities found in firstMatch"_s)));
            return { };
        }
        // 5.1. Let validated capabilities be the result of trying to validate capabilities with argument first match capabilities.
        firstMatchCapabilities = validatedCapabilities(*firstMatchCapabilities);
        if (!firstMatchCapabilities) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, String("Invalid firstMatch capabilities"_s)));
            return { };
        }

        // Validate here that firstMatchCapabilities don't shadow alwaysMatchCapabilities.
        auto requiredEnd = requiredCapabilities->end();
        auto firstMatchEnd = firstMatchCapabilities->end();
        for (auto it = firstMatchCapabilities->begin(); it != firstMatchEnd; ++it) {
            if (requiredCapabilities->find(it->key) != requiredEnd) {
                completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument,
                    makeString("Invalid firstMatch capabilities: key "_s, it->key, " is present in alwaysMatch"_s)));
                return { };
            }
        }

        // 5.2. Append validated capabilities to validated first match capabilities.
        validatedFirstMatchCapabilitiesList.append(WTFMove(firstMatchCapabilities));
    }

    // 6. For each first match capabilities corresponding to an indexed property in validated first match capabilities.
    Vector<Capabilities> matchedCapabilitiesList;
    matchedCapabilitiesList.reserveInitialCapacity(validatedFirstMatchCapabilitiesList.size());
    for (auto& validatedFirstMatchCapabilies : validatedFirstMatchCapabilitiesList) {
        // 6.1. Let merged capabilities be the result of trying to merge capabilities with required capabilities and first match capabilities as arguments.
        auto mergedCapabilities = mergeCapabilities(*requiredCapabilities, *validatedFirstMatchCapabilies);

        // 6.2. Let matched capabilities be the result of trying to match capabilities with merged capabilities as an argument.
        if (auto matchedCapabilities = matchCapabilities(*mergedCapabilities)) {
            // 6.3. If matched capabilities is not null return matched capabilities.
            Capabilities capabilities;
            parseCapabilities(*matchedCapabilities, capabilities);
            matchedCapabilitiesList.append(WTFMove(capabilities));
        }
    }

    if (matchedCapabilitiesList.isEmpty()) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::SessionNotCreated, String("Failed to match capabilities"_s)));
        return { };
    }

    return matchedCapabilitiesList;
}

void WebDriverService::newSession(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §8.1 New Session.
    // https://www.w3.org/TR/webdriver/#new-session
    auto matchedCapabilitiesList = processCapabilities(*parameters, completionHandler);
    if (matchedCapabilitiesList.isEmpty())
        return;

    if (!m_session) {
        // Reverse the vector to always take last item.
        matchedCapabilitiesList.reverse();
        connectToBrowser(WTFMove(matchedCapabilitiesList), WTFMove(completionHandler));
        return;
    }

    if (m_replaceOnNewSession) {
        RELEASE_LOG(WebDriverClassic, "WebDriverService::newSession: Replacing existing session.");
        auto session = std::exchange(m_session, nullptr);
        session->close([this, session, matchedCapabilitiesList, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
#if ENABLE(WEBDRIVER_BIDI)
            m_bidiServer->disconnectSession(session->id());
#endif
            // Ignore unknown errors when closing the session if the session has abeen actually closed.
            if ((!result.isError()) || (result.errorCode() == CommandResult::ErrorCode::UnknownError && !session->isConnected())) {
                matchedCapabilitiesList.reverse();
                connectToBrowser(WTFMove(matchedCapabilitiesList), WTFMove(completionHandler));
            } else
                completionHandler(WTFMove(result));
        });
        return;
    }
    RELEASE_LOG(WebDriverClassic, "WebDriverService::newSession: Maximum number of active sessions reached. Returning error.");
    completionHandler(CommandResult::fail(CommandResult::ErrorCode::SessionNotCreated, String("Maximum number of active sessions"_s)));
}

void WebDriverService::connectToBrowser(Vector<Capabilities>&& capabilitiesList, Function<void (CommandResult&&)>&& completionHandler)
{
    if (capabilitiesList.isEmpty()) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::SessionNotCreated, String("Failed to match capabilities"_s)));
        return;
    }

    auto sessionHost = SessionHost::create(capabilitiesList.takeLast());
    sessionHost->setHostAddress(m_targetAddress, m_targetPort);
    auto protectedSessionHost = Ref<SessionHost>(sessionHost);
    protectedSessionHost->connectToBrowser([this, capabilitiesList = WTFMove(capabilitiesList), sessionHost = WTFMove(sessionHost), completionHandler = WTFMove(completionHandler)](std::optional<String> error) mutable {
        if (error) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::SessionNotCreated, makeString("Failed to connect to browser: "_s, error.value())));
            return;
        }

        createSession(WTFMove(capabilitiesList), WTFMove(sessionHost), WTFMove(completionHandler));
    });
}

void WebDriverService::createSession(Vector<Capabilities>&& capabilitiesList, Ref<SessionHost>&& sessionHost, Function<void (CommandResult&&)>&& completionHandler)
{
    auto protectedSessionHost = Ref<SessionHost>(sessionHost);
    protectedSessionHost->startAutomationSession([this, capabilitiesList = WTFMove(capabilitiesList), sessionHost = WTFMove(sessionHost), completionHandler = WTFMove(completionHandler)](bool capabilitiesDidMatch, std::optional<String> errorMessage) mutable {
        if (errorMessage) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::UnknownError, errorMessage.value()));
            return;
        }
        if (!capabilitiesDidMatch) {
            connectToBrowser(WTFMove(capabilitiesList), WTFMove(completionHandler));
            return;
        }
#if ENABLE(WEBDRIVER_BIDI)
        RefPtr<Session> session = Session::create(WTFMove(sessionHost), m_bidiServer);
#else
        RefPtr<Session> session = Session::create(WTFMove(sessionHost));
#endif
        session->createTopLevelBrowsingContext([this, session, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
            if (result.isError()) {
                completionHandler(CommandResult::fail(CommandResult::ErrorCode::SessionNotCreated, result.errorMessage().value_or("Unknown error creating top level browsing context."_s)));
                return;
            }

            m_session = WTFMove(session);

            auto resultObject = JSON::Object::create();
            resultObject->setString("sessionId"_s, m_session->id());
            auto capabilitiesObject = JSON::Object::create();
            const auto& capabilities = m_session->capabilities();
            capabilitiesObject->setString("browserName"_s, capabilities.browserName.value_or(emptyString()));
            capabilitiesObject->setString("browserVersion"_s, capabilities.browserVersion.value_or(emptyString()));
            capabilitiesObject->setString("platformName"_s, capabilities.platformName.value_or(emptyString()));
            capabilitiesObject->setBoolean("acceptInsecureCerts"_s, capabilities.acceptInsecureCerts.value_or(false));
            capabilitiesObject->setBoolean("strictFileInteractability"_s, capabilities.strictFileInteractability.value_or(false));
            capabilitiesObject->setBoolean("setWindowRect"_s, capabilities.setWindowRect.value_or(true));
            switch (capabilities.unhandledPromptBehavior.value_or(UnhandledPromptBehavior::DismissAndNotify)) {
            case UnhandledPromptBehavior::Dismiss:
                capabilitiesObject->setString("unhandledPromptBehavior"_s, "dismiss"_s);
                break;
            case UnhandledPromptBehavior::Accept:
                capabilitiesObject->setString("unhandledPromptBehavior"_s, "accept"_s);
                break;
            case UnhandledPromptBehavior::DismissAndNotify:
                capabilitiesObject->setString("unhandledPromptBehavior"_s, "dismiss and notify"_s);
                break;
            case UnhandledPromptBehavior::AcceptAndNotify:
                capabilitiesObject->setString("unhandledPromptBehavior"_s, "accept and notify"_s);
                break;
            case UnhandledPromptBehavior::Ignore:
                capabilitiesObject->setString("unhandledPromptBehavior"_s, "ignore"_s);
                break;
            }
            switch (capabilities.pageLoadStrategy.value_or(PageLoadStrategy::Normal)) {
            case PageLoadStrategy::None:
                capabilitiesObject->setString("pageLoadStrategy"_s, "none"_s);
                break;
            case PageLoadStrategy::Normal:
                capabilitiesObject->setString("pageLoadStrategy"_s, "normal"_s);
                break;
            case PageLoadStrategy::Eager:
                capabilitiesObject->setString("pageLoadStrategy"_s, "eager"_s);
                break;
            }
            if (!capabilities.proxy)
                capabilitiesObject->setObject("proxy"_s, JSON::Object::create());
            auto timeoutsObject = JSON::Object::create();
            if (m_session->scriptTimeout() == std::numeric_limits<double>::infinity())
                timeoutsObject->setValue("script"_s, JSON::Value::null());
            else
                timeoutsObject->setDouble("script"_s, m_session->scriptTimeout());
            timeoutsObject->setDouble("pageLoad"_s, m_session->pageLoadTimeout());
            timeoutsObject->setDouble("implicit"_s, m_session->implicitWaitTimeout());
            capabilitiesObject->setObject("timeouts"_s, WTFMove(timeoutsObject));

#if ENABLE(WEBDRIVER_BIDI)
            // Extension steps defined by BiDi spec: https://w3c.github.io/webdriver-bidi/#establishing
            if (!m_session->hasBiDiEnabled() && capabilities.webSocketURL && *capabilities.webSocketURL) {
                auto listener = m_bidiServer->startListening(m_session->id());
                // We need to update the listener host to a visible one so remote clients can connect to it.
                listener->host = m_server.visibleHost();

                auto webSocketURL = m_bidiServer->getWebSocketURL(listener, m_session->id());
                capabilitiesObject->setString("webSocketUrl"_s, webSocketURL);
                m_session->setHasBiDiEnabled(true);
            } else {
                RELEASE_LOG(WebDriverBiDi, "BiDi support not enabled for session %s", m_session->id().utf8().data());
                if (!m_session->hasBiDiEnabled())
                    RELEASE_LOG(WebDriverBiDi, "BiDi flag not set for session %s", m_session->id().utf8().data());
                if (!capabilities.webSocketURL || !*capabilities.webSocketURL)
                    RELEASE_LOG(WebDriverBiDi, "webSocketURL not set for session %s", m_session->id().utf8().data());
            }
#endif

            resultObject->setObject("capabilities"_s, WTFMove(capabilitiesObject));
            completionHandler(CommandResult::success(WTFMove(resultObject)));
        });
    });
}

void WebDriverService::deleteSession(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §8.2 Delete Session.
    // https://www.w3.org/TR/webdriver/#delete-session
    auto sessionID = parameters->getString("sessionId"_s);
    if (!sessionID) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing session ID parameter"_s));
        return;
    }

    if (!m_session || m_session->id() != sessionID) {
        completionHandler(CommandResult::success());
        return;
    }

    auto session = std::exchange(m_session, nullptr);
    session->close([this, session, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        UNUSED_VARIABLE(this); // Conditionally used in ENABLE(WEBDRIVER_BIDI) block.
#if ENABLE(WEBDRIVER_BIDI)
        m_bidiServer->disconnectSession(session->id());
#endif
        // Ignore unknown errors when closing the session if the session has abeen actually closed.
        if (result.isError() && result.errorCode() == CommandResult::ErrorCode::UnknownError && !session->isConnected())
            completionHandler(CommandResult::success());
        else
            completionHandler(WTFMove(result));
    });
}

void WebDriverService::status(RefPtr<JSON::Object>&&, Function<void (CommandResult&&)>&& completionHandler)
{
    // §8.3 Status
    // https://w3c.github.io/webdriver/webdriver-spec.html#status
    auto body = JSON::Object::create();
    body->setBoolean("ready"_s, !m_session);
    body->setString("message"_s, m_session ? "A session already exists"_s : "No sessions"_s);
    completionHandler(CommandResult::success(WTFMove(body)));
}

void WebDriverService::getTimeouts(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §8.4 Get Timeouts.
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-timeouts
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->getTimeouts(WTFMove(completionHandler));
}

void WebDriverService::setTimeouts(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §8.5 Set Timeouts.
    // https://www.w3.org/TR/webdriver/#set-timeouts
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto timeouts = deserializeTimeouts(*parameters, IgnoreUnknownTimeout::Yes);
    if (!timeouts) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid timeouts"_s));
        return;
    }

    m_session->setTimeouts(timeouts.value(), WTFMove(completionHandler));
}

void WebDriverService::go(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §9.1 Go.
    // https://www.w3.org/TR/webdriver/#go
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto url = parameters->getString("url"_s);
    if (!url) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing URL parameter"_s));
        return;
    }

    m_session->waitForNavigationToComplete([this, url = WTFMove(url), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->go(url, WTFMove(completionHandler));
    });
}

void WebDriverService::getCurrentURL(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §9.2 Get Current URL.
    // https://www.w3.org/TR/webdriver/#get-current-url
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->getCurrentURL(WTFMove(completionHandler));
    });
}

void WebDriverService::back(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §9.3 Back.
    // https://www.w3.org/TR/webdriver/#back
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->back(WTFMove(completionHandler));
    });
}

void WebDriverService::forward(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §9.4 Forward.
    // https://www.w3.org/TR/webdriver/#forward
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->forward(WTFMove(completionHandler));
    });
}

void WebDriverService::refresh(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §9.5 Refresh.
    // https://www.w3.org/TR/webdriver/#refresh
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->refresh(WTFMove(completionHandler));
    });
}

void WebDriverService::getTitle(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §9.6 Get Title.
    // https://www.w3.org/TR/webdriver/#get-title
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->getTitle(WTFMove(completionHandler));
    });
}

void WebDriverService::getWindowHandle(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.1 Get Window Handle.
    // https://www.w3.org/TR/webdriver/#get-window-handle
    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->getWindowHandle(WTFMove(completionHandler));
}

void WebDriverService::getWindowRect(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.7.1 Get Window Rect.
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-window-rect
    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->getWindowRect(WTFMove(completionHandler));
}

void WebDriverService::setWindowRect(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.7.2 Set Window Rect.
    // https://w3c.github.io/webdriver/webdriver-spec.html#set-window-rect
    std::optional<double> width;
    if (auto value = parameters->getValue("width"_s)) {
        if (auto number = valueAsNumberInRange(*value))
            width = number;
        else if (!value->isNull()) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid width value"_s));
            return;
        }
    }
    std::optional<double> height;
    if (auto value = parameters->getValue("height"_s)) {
        if (auto number = valueAsNumberInRange(*value))
            height = number;
        else if (!value->isNull()) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid height value"_s));
            return;
        }
    }
    std::optional<double> x;
    if (auto value = parameters->getValue("x"_s)) {
        if (auto number = valueAsNumberInRange(*value, INT_MIN))
            x = number;
        else if (!value->isNull()) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid x value"_s));
            return;
        }
    }
    std::optional<double> y;
    if (auto value = parameters->getValue("y"_s)) {
        if (auto number = valueAsNumberInRange(*value, INT_MIN))
            y = number;
        else if (!value->isNull()) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid y value"_s));
            return;
        }
    }

    // FIXME: If the remote end does not support the Set Window Rect command for the current
    // top-level browsing context for any reason, return error with error code unsupported operation.

    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->setWindowRect(x, y, width, height, WTFMove(completionHandler));
}

void WebDriverService::maximizeWindow(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.7.3 Maximize Window
    // https://w3c.github.io/webdriver/#maximize-window
    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->maximizeWindow(WTFMove(completionHandler));
}

void WebDriverService::minimizeWindow(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.7.4 Minimize Window
    // https://w3c.github.io/webdriver/#minimize-window
    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->minimizeWindow(WTFMove(completionHandler));
}

void WebDriverService::fullscreenWindow(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.7.5 Fullscreen Window
    // https://w3c.github.io/webdriver/#fullscreen-window
    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->fullscreenWindow(WTFMove(completionHandler));
}

void WebDriverService::closeWindow(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.2 Close Window.
    // https://www.w3.org/TR/webdriver/#close-window
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->closeWindow([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }

        auto handles = result.result()->asArray();
        if (handles && !handles->length())
            m_session = nullptr;

        completionHandler(WTFMove(result));
    });
}

void WebDriverService::switchToWindow(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.3 Switch To Window.
    // https://www.w3.org/TR/webdriver/#switch-to-window
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto handle = parameters->getString("handle"_s);
    if (!handle) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing handle parameter"_s));
        return;
    }

    m_session->switchToWindow(handle, WTFMove(completionHandler));
}

void WebDriverService::getWindowHandles(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.4 Get Window Handles.
    // https://www.w3.org/TR/webdriver/#get-window-handles
    if (findSessionOrCompleteWithError(*parameters, completionHandler))
        m_session->getWindowHandles(WTFMove(completionHandler));
}

void WebDriverService::newWindow(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §11.5 New Window
    // https://w3c.github.io/webdriver/#new-window
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    std::optional<String> typeHint;
    if (auto value = parameters->getValue("type"_s)) {
        auto valueString = value->asString();
        if (!!valueString) {
            if (valueString == "window"_s || valueString == "tab"_s)
                typeHint = valueString;
        } else if (!value->isNull()) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid type value"_s));
            return;
        }
    }

    m_session->newWindow(typeHint, WTFMove(completionHandler));
}

void WebDriverService::switchToFrame(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.5 Switch To Frame.
    // https://www.w3.org/TR/webdriver/#switch-to-frame
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto frameID = parameters->getValue("id"_s);
    if (!frameID) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing frame ID parameter"_s));
        return;
    }

    switch (frameID->type()) {
    case JSON::Value::Type::Null:
        break;
    case JSON::Value::Type::Double:
    case JSON::Value::Type::Integer:
        if (!valueAsNumberInRange(*frameID, 0, std::numeric_limits<unsigned short>::max())) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid frame ID value"_s));
            return;
        }
        break;
    case JSON::Value::Type::Object: {
        auto frameIDObject = frameID->asObject();
        if (frameIDObject->find(Session::webElementIdentifier()) == frameIDObject->end()) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid frame ID object"_s));
            return;
        }
        break;
    }
    case JSON::Value::Type::Boolean:
    case JSON::Value::Type::String:
    case JSON::Value::Type::Array:
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid frame ID type"_s));
        return;
    }

    m_session->waitForNavigationToComplete([this, frameID, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->switchToFrame(WTFMove(frameID), WTFMove(completionHandler));
    });
}

void WebDriverService::switchToParentFrame(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §10.6 Switch To Parent Frame.
    // https://www.w3.org/TR/webdriver/#switch-to-parent-frame
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->switchToParentFrame(WTFMove(completionHandler));
    });
}

static std::optional<String> findElementOrCompleteWithError(JSON::Object& parameters, Function<void (CommandResult&&)>& completionHandler, Session::ElementIsShadowRoot isShadowRoot = Session::ElementIsShadowRoot::No)
{
    auto elementID = parameters.getString(isShadowRoot == Session::ElementIsShadowRoot::Yes ? "shadowId"_s : "elementId"_s);
    if (elementID.isEmpty()) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing element ID or shadow ID parameter"_s));
        return std::nullopt;
    }
    return elementID;
}

static inline bool isValidStrategy(const String& strategy)
{
    // §12.1 Locator Strategies.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dfn-table-of-location-strategies
    return strategy == "css selector"_s
        || strategy == "link text"_s
        || strategy == "partial link text"_s
        || strategy == "tag name"_s
        || strategy == "xpath"_s;
}

static bool findStrategyAndSelectorOrCompleteWithError(JSON::Object& parameters, Function<void (CommandResult&&)>& completionHandler, Session::ElementIsShadowRoot isShadowRoot, String& strategy, String& selector)
{
    strategy = parameters.getString("using"_s);
    if (!isValidStrategy(strategy)) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, makeString("Invalid strategy: "_s, strategy)));
        return false;
    }
    selector = parameters.getString("value"_s);
    if (!selector) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing selector value"_s));
        return false;
    }

    if (isShadowRoot == Session::ElementIsShadowRoot::Yes) {
        // Currently there is an opened discussion about if the following values has to be supported for a Shadow Root
        // because the current implementation doesn't support them. We have them disabled for now.
        // https://github.com/w3c/webdriver/issues/1610
        if (strategy == "tag name"_s || strategy == "xpath"_s) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidSelector, makeString("Unsupported strategy for shadow root: "_s, strategy)));
            return false;
        }
    }

    return true;
}

void WebDriverService::findElement(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.2 Find Element.
    // https://www.w3.org/TR/webdriver/#find-element
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    String strategy, selector;
    if (!findStrategyAndSelectorOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::No, strategy, selector))
        return;

    m_session->waitForNavigationToComplete([this, strategy = WTFMove(strategy), selector = WTFMove(selector), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->findElements(strategy, selector, Session::FindElementsMode::Single, emptyString(), Session::ElementIsShadowRoot::No, WTFMove(completionHandler));
    });
}

void WebDriverService::findElements(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.3 Find Elements.
    // https://www.w3.org/TR/webdriver/#find-elements
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    String strategy, selector;
    if (!findStrategyAndSelectorOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::No, strategy, selector))
        return;

    m_session->waitForNavigationToComplete([this, strategy = WTFMove(strategy), selector = WTFMove(selector), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->findElements(strategy, selector, Session::FindElementsMode::Multiple, emptyString(), Session::ElementIsShadowRoot::No, WTFMove(completionHandler));
    });
}

void WebDriverService::findElementFromElement(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.4 Find Element From Element.
    // https://www.w3.org/TR/webdriver/#find-element-from-element
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    String strategy, selector;
    if (!findStrategyAndSelectorOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::No, strategy, selector))
        return;
    m_session->findElements(strategy, selector, Session::FindElementsMode::Single, elementID.value(), Session::ElementIsShadowRoot::No, WTFMove(completionHandler));
}

void WebDriverService::findElementsFromElement(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.5 Find Elements From Element.
    // https://www.w3.org/TR/webdriver/#find-elements-from-element
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    String strategy, selector;
    if (!findStrategyAndSelectorOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::No, strategy, selector))
        return;

    m_session->findElements(strategy, selector, Session::FindElementsMode::Multiple, elementID.value(), Session::ElementIsShadowRoot::No, WTFMove(completionHandler));
}

void WebDriverService::findElementFromShadowRoot(RefPtr<JSON::Object>&& parameters, Function<void(CommandResult&&)>&& completionHandler)
{
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto shadowID = findElementOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::Yes);
    if (!shadowID)
        return;

    String strategy, selector;
    if (!findStrategyAndSelectorOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::Yes, strategy, selector))
        return;

    m_session->findElements(strategy, selector, Session::FindElementsMode::Single, shadowID.value(), Session::ElementIsShadowRoot::Yes, WTFMove(completionHandler));
}

void WebDriverService::findElementsFromShadowRoot(RefPtr<JSON::Object>&& parameters, Function<void(CommandResult&&)>&& completionHandler)
{
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto shadowID = findElementOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::Yes);
    if (!shadowID)
        return;

    String strategy, selector;
    if (!findStrategyAndSelectorOrCompleteWithError(*parameters, completionHandler, Session::ElementIsShadowRoot::Yes, strategy, selector))
        return;

    m_session->findElements(strategy, selector, Session::FindElementsMode::Multiple, shadowID.value(), Session::ElementIsShadowRoot::Yes, WTFMove(completionHandler));
}

void WebDriverService::getActiveElement(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.6 Get Active Element.
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-active-element
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->getActiveElement(WTFMove(completionHandler));
    });
}

void WebDriverService::getElementShadowRoot(RefPtr<JSON::Object>&& parameters, Function<void(CommandResult&&)>&& completionHandler)
{
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->getElementShadowRoot(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::isElementSelected(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.1 Is Element Selected.
    // https://www.w3.org/TR/webdriver/#is-element-selected
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->isElementSelected(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::getElementAttribute(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.2 Get Element Attribute.
    // https://www.w3.org/TR/webdriver/#get-element-attribute
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    auto attribute = parameters->getString("name"_s);
    if (!attribute) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing attribute name parameter"_s));
        return;
    }

    m_session->getElementAttribute(elementID.value(), attribute, WTFMove(completionHandler));
}

void WebDriverService::getElementProperty(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.3 Get Element Property
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-element-property
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    auto attribute = parameters->getString("name"_s);
    if (!attribute) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing property name parameter"_s));
        return;
    }

    m_session->getElementProperty(elementID.value(), attribute, WTFMove(completionHandler));
}

void WebDriverService::getElementCSSValue(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.4 Get Element CSS Value
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-element-css-value
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    auto cssProperty = parameters->getString("name"_s);
    if (!cssProperty) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing CSS property name parameter"_s));
        return;
    }

    m_session->getElementCSSValue(elementID.value(), cssProperty, WTFMove(completionHandler));
}

void WebDriverService::getElementText(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.5 Get Element Text.
    // https://www.w3.org/TR/webdriver/#get-element-text
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->getElementText(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::getElementTagName(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.6 Get Element Tag Name.
    // https://www.w3.org/TR/webdriver/#get-element-tag-name
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->getElementTagName(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::getElementRect(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.7 Get Element Rect.
    // https://www.w3.org/TR/webdriver/#get-element-rect
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->getElementRect(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::isElementEnabled(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §13.8 Is Element Enabled.
    // https://www.w3.org/TR/webdriver/#is-element-enabled
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->isElementEnabled(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::getComputedRole(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.4.9 Get Computed Role
    // https://www.w3.org/TR/webdriver/#get-computed-role
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->getComputedRole(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::getComputedLabel(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §12.4.10 Get Computed Role
    // https://www.w3.org/TR/webdriver/#get-computed-label
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->getComputedLabel(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::isElementDisplayed(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §C. Element Displayedness.
    // https://www.w3.org/TR/webdriver/#element-displayedness
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->isElementDisplayed(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::elementClick(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §14.1 Element Click.
    // https://www.w3.org/TR/webdriver/#element-click
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->elementClick(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::elementClear(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §14.2 Element Clear.
    // https://www.w3.org/TR/webdriver/#element-clear
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->elementClear(elementID.value(), WTFMove(completionHandler));
}

void WebDriverService::elementSendKeys(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §14.3 Element Send Keys.
    // https://www.w3.org/TR/webdriver/#element-send-keys
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    auto text = parameters->getString("text"_s);
    if (text.isEmpty()) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing text parameter"_s));
        return;
    }

    m_session->elementSendKeys(elementID.value(), text, WTFMove(completionHandler));
}

void WebDriverService::getPageSource(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §15.1 Getting Page Source.
    // https://w3c.github.io/webdriver/webdriver-spec.html#getting-page-source
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->getPageSource(WTFMove(completionHandler));
}

static bool findScriptAndArgumentsOrCompleteWithError(JSON::Object& parameters, Function<void (CommandResult&&)>& completionHandler, String& script, RefPtr<JSON::Array>& arguments)
{
    script = parameters.getString("script"_s);
    if (!script) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing script parameter"_s));
        return false;
    }
    arguments = parameters.getArray("args"_s);
    if (!arguments) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing args parameter"_s));
        return false;
    }
    return true;
}

void WebDriverService::executeScript(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §15.2.1 Execute Script.
    // https://www.w3.org/TR/webdriver/#execute-script
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    String script;
    RefPtr<JSON::Array> arguments;
    if (!findScriptAndArgumentsOrCompleteWithError(*parameters, completionHandler, script, arguments))
        return;

    m_session->waitForNavigationToComplete([this, script = WTFMove(script), arguments = WTFMove(arguments), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->executeScript(script, WTFMove(arguments), Session::ExecuteScriptMode::Sync, WTFMove(completionHandler));
    });
}

void WebDriverService::executeAsyncScript(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §15.2.2 Execute Async Script.
    // https://www.w3.org/TR/webdriver/#execute-async-script
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    String script;
    RefPtr<JSON::Array> arguments;
    if (!findScriptAndArgumentsOrCompleteWithError(*parameters, completionHandler, script, arguments))
        return;

    m_session->waitForNavigationToComplete([this, script = WTFMove(script), arguments = WTFMove(arguments), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->executeScript(script, WTFMove(arguments), Session::ExecuteScriptMode::Async, WTFMove(completionHandler));
    });
}

void WebDriverService::getAllCookies(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §16.1 Get All Cookies.
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-all-cookies
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->getAllCookies(WTFMove(completionHandler));
    });
}

void WebDriverService::getNamedCookie(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §16.2 Get Named Cookie.
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-named-cookie
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto name = parameters->getString("name"_s);
    if (!name) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing cookie name parameter"_s));
        return;
    }

    m_session->waitForNavigationToComplete([this, name = WTFMove(name), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->getNamedCookie(name, WTFMove(completionHandler));
    });
}

static std::optional<Session::Cookie> deserializeCookie(JSON::Object& cookieObject)
{
    Session::Cookie cookie;

    cookie.name = cookieObject.getString("name"_s);
    if (cookie.name.isEmpty())
        return std::nullopt;

    cookie.value = cookieObject.getString("value"_s);
    if (cookie.value.isEmpty())
        return std::nullopt;

    if (auto value = cookieObject.getValue("path"_s)) {
        auto path = value->asString();
        if (!path)
            return std::nullopt;
        cookie.path = path;
    }
    if (auto value = cookieObject.getValue("domain"_s)) {
        auto domain = value->asString();
        if (!domain)
            return std::nullopt;
        cookie.domain = domain;
    }
    if (auto value = cookieObject.getValue("secure"_s)) {
        auto secure = value->asBoolean();
        if (!secure)
            return std::nullopt;
        cookie.secure = secure;
    }
    if (auto value = cookieObject.getValue("httpOnly"_s)) {
        auto httpOnly = value->asBoolean();
        if (!httpOnly)
            return std::nullopt;
        cookie.httpOnly = httpOnly;
    }
    if (auto value = cookieObject.getValue("expiry"_s)) {
        auto expiry = unsignedValue(*value);
        if (!expiry)
            return std::nullopt;
        cookie.expiry = expiry.value();
    }
    if (auto value = cookieObject.getValue("sameSite"_s)) {
        auto sameSite = value->asString();
        if (sameSite != "None"_s && sameSite != "Lax"_s && sameSite != "Strict"_s)
            return std::nullopt;
        cookie.sameSite = sameSite;
    }

    return cookie;
}

void WebDriverService::addCookie(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §16.3 Add Cookie.
    // https://w3c.github.io/webdriver/webdriver-spec.html#add-cookie
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto cookieObject = parameters->getObject("cookie"_s);
    if (!cookieObject) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing cookie parameter"_s));
        return;
    }

    auto cookie = deserializeCookie(*cookieObject);
    if (!cookie) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Invalid cookie parameter"_s));
        return;
    }

    m_session->waitForNavigationToComplete([this, cookie = WTFMove(cookie), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->addCookie(cookie.value(), WTFMove(completionHandler));
    });
}

void WebDriverService::deleteCookie(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §16.4 Delete Cookie.
    // https://w3c.github.io/webdriver/webdriver-spec.html#delete-cookie
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto name = parameters->getString("name"_s);
    if (!name) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing cookie name parameter"_s));
        return;
    }

    m_session->waitForNavigationToComplete([this, name = WTFMove(name), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->deleteCookie(name, WTFMove(completionHandler));
    });
}

void WebDriverService::deleteAllCookies(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §16.5 Delete All Cookies.
    // https://w3c.github.io/webdriver/webdriver-spec.html#delete-all-cookies
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->deleteAllCookies(WTFMove(completionHandler));
    });
}

static bool processPauseAction(JSON::Object& actionItem, Action& action, std::optional<String>& errorMessage)
{
    auto durationValue = actionItem.getValue("duration"_s);
    if (!durationValue)
        return true;

    auto duration = unsignedValue(*durationValue);
    if (!duration) {
        errorMessage = String("The 'duration' parameter for the pause action is invalid"_s);
        return false;
    }

    action.duration = duration.value();
    return true;
}

static std::optional<Action> processNullAction(const String& id, JSON::Object& actionItem, std::optional<String>& errorMessage)
{
    auto subtype = actionItem.getString("type"_s);
    if (subtype != "pause"_s) {
        errorMessage = String("The 'type' parameter for the null action is invalid or missing"_s);
        return std::nullopt;
    }

    Action action(id, Action::Type::None, Action::Subtype::Pause);
    if (!processPauseAction(actionItem, action, errorMessage))
        return std::nullopt;

    return action;
}

static std::optional<Action> processKeyAction(const String& id, JSON::Object& actionItem, std::optional<String>& errorMessage)
{
    Action::Subtype actionSubtype;
    auto subtype = actionItem.getString("type"_s);
    if (subtype == "pause"_s)
        actionSubtype = Action::Subtype::Pause;
    else if (subtype == "keyUp"_s)
        actionSubtype = Action::Subtype::KeyUp;
    else if (subtype == "keyDown"_s)
        actionSubtype = Action::Subtype::KeyDown;
    else {
        errorMessage = String("The 'type' parameter for the key action is invalid"_s);
        return std::nullopt;
    }

    Action action(id, Action::Type::Key, actionSubtype);

    switch (actionSubtype) {
    case Action::Subtype::Pause:
        if (!processPauseAction(actionItem, action, errorMessage))
            return std::nullopt;
        break;
    case Action::Subtype::KeyUp:
    case Action::Subtype::KeyDown: {
        auto keyValue = actionItem.getValue("value"_s);
        if (!keyValue) {
            errorMessage = String("The 'value' parameter for the key up/down action is missing "_s);
            return std::nullopt;
        }
        auto key = keyValue->asString();
        if (key.isEmpty()) {
            errorMessage = String("The 'value' parameter for the key up/down action is invalid"_s);
            return std::nullopt;
        }
        // FIXME: check single unicode code point.
        action.key = key;
        break;
    }
    case Action::Subtype::PointerUp:
    case Action::Subtype::PointerDown:
    case Action::Subtype::PointerMove:
    case Action::Subtype::PointerCancel:
    case Action::Subtype::Scroll:
        ASSERT_NOT_REACHED();
    }

    return action;
}

static MouseButton actionMouseButton(unsigned button)
{
    // MouseEvent.button
    // https://www.w3.org/TR/uievents/#ref-for-dom-mouseevent-button-1
    switch (button) {
    case 0:
        return MouseButton::Left;
    case 1:
        return MouseButton::Middle;
    case 2:
        return MouseButton::Right;
    }

    return MouseButton::None;
}

enum class CoordinateType { Fractional, Integral };

static bool processPointerMoveAction(JSON::Object& actionItem, Action& action, std::optional<String>& errorMessage, CoordinateType coordinateType = CoordinateType::Fractional)
{
    if (auto durationValue = actionItem.getValue("duration"_s)) {
        auto duration = unsignedValue(*durationValue);
        if (!duration) {
            errorMessage = String("The 'duration' parameter for the action is invalid"_s);
            return false;
        }
        action.duration = duration.value();
    }

    if (auto originValue = actionItem.getValue("origin"_s)) {
        if (auto originObject = originValue->asObject()) {
            auto elementID = originObject->getString(Session::webElementIdentifier());
            if (!elementID) {
                errorMessage = String("The 'origin' parameter for the action is not a valid web element object"_s);
                return false;
            }
            action.origin = PointerOrigin { PointerOrigin::Type::Element, elementID };
        } else {
            auto origin = originValue->asString();
            if (origin == "viewport"_s)
                action.origin = PointerOrigin { PointerOrigin::Type::Viewport, std::nullopt };
            else if (origin == "pointer"_s)
                action.origin = PointerOrigin { PointerOrigin::Type::Pointer, std::nullopt };
            else {
                errorMessage = String("The 'origin' parameter for the action is invalid"_s);
                return false;
            }
        }
    } else
        action.origin = PointerOrigin { PointerOrigin::Type::Viewport, std::nullopt };

    if (auto xValue = actionItem.getValue("x"_s)) {
        auto x = valueAsNumberInRange(*xValue, INT_MIN);
        if (!x || (coordinateType == CoordinateType::Integral && x.value() != std::floor(x.value()))) {
            errorMessage = String("The 'x' parameter for the action is invalid"_s);
            return false;
        }
        action.x = x.value();
    } else {
        errorMessage = String("The 'x' parameter for the action is missing"_s);
        return false;
    }

    if (auto yValue = actionItem.getValue("y"_s)) {
        auto y = valueAsNumberInRange(*yValue, INT_MIN);
        if (!y || (coordinateType == CoordinateType::Integral && y.value() != std::floor(y.value()))) {
            errorMessage = String("The 'y' parameter for the action is invalid"_s);
            return false;
        }
        action.y = y.value();
    } else {
        errorMessage = String("The 'y' parameter for the action is missing"_s);
        return false;
    }

    return true;
}

static std::optional<Action> processPointerAction(const String& id, PointerParameters& parameters, JSON::Object& actionItem, std::optional<String>& errorMessage)
{
    Action::Subtype actionSubtype;
    auto subtype = actionItem.getString("type"_s);
    if (subtype == "pause"_s)
        actionSubtype = Action::Subtype::Pause;
    else if (subtype == "pointerUp"_s)
        actionSubtype = Action::Subtype::PointerUp;
    else if (subtype == "pointerDown"_s)
        actionSubtype = Action::Subtype::PointerDown;
    else if (subtype == "pointerMove"_s)
        actionSubtype = Action::Subtype::PointerMove;
    else if (subtype == "pointerCancel"_s)
        actionSubtype = Action::Subtype::PointerCancel;
    else {
        errorMessage = String("The 'type' parameter for the pointer action is invalid"_s);
        return std::nullopt;
    }

    Action action(id, Action::Type::Pointer, actionSubtype);
    action.pointerType = parameters.pointerType;

    switch (actionSubtype) {
    case Action::Subtype::Pause:
        if (!processPauseAction(actionItem, action, errorMessage))
            return std::nullopt;
        break;
    case Action::Subtype::PointerUp:
    case Action::Subtype::PointerDown: {
        auto buttonValue = actionItem.getValue("button"_s);
        if (!buttonValue) {
            errorMessage = String("The 'button' parameter for the pointer up/down action is missing"_s);
            return std::nullopt;
        }
        auto button = unsignedValue(*buttonValue);
        if (!button) {
            errorMessage = String("The 'button' parameter for the pointer up/down action is invalid"_s);
            return std::nullopt;
        }
        action.button = actionMouseButton(button.value());
        break;
    }
    case Action::Subtype::PointerMove:
        if (!processPointerMoveAction(actionItem, action, errorMessage))
            return std::nullopt;
        break;
    case Action::Subtype::PointerCancel:
        break;
    case Action::Subtype::KeyUp:
    case Action::Subtype::KeyDown:
    case Action::Subtype::Scroll:
        ASSERT_NOT_REACHED();
    }

    return action;
}

static std::optional<Action> processWheelAction(const String& id, JSON::Object& actionItem, std::optional<String>& errorMessage)
{
    Action::Subtype actionSubtype;
    auto subtype = actionItem.getString("type"_s);
    if (subtype == "pause"_s)
        actionSubtype = Action::Subtype::Pause;
    else if (subtype == "scroll"_s)
        actionSubtype = Action::Subtype::Scroll;
    else {
        errorMessage = String("The 'type' parameter for the wheel action is invalid"_s);
        return std::nullopt;
    }

    Action action(id, Action::Type::Wheel, actionSubtype);

    switch (actionSubtype) {
    case Action::Subtype::Pause:
        if (!processPauseAction(actionItem, action, errorMessage))
            return std::nullopt;
        break;
    case Action::Subtype::Scroll:
        if (!processPointerMoveAction(actionItem, action, errorMessage, CoordinateType::Integral))
            return std::nullopt;

        if (auto deltaXValue = actionItem.getValue("deltaX"_s)) {
            auto deltaX = valueAsNumberInRange(*deltaXValue, INT_MIN);
            if (!deltaX || deltaX.value() != std::floor(deltaX.value())) {
                errorMessage = String("The 'deltaX' parameter for the action is invalid"_s);
                return std::nullopt;
            }
            action.deltaX = deltaX.value();
        } else {
            errorMessage = String("The 'deltaX' parameter for the action is missing"_s);
            return std::nullopt;
        }

        if (auto deltaYValue = actionItem.getValue("deltaY"_s)) {
            auto deltaY = valueAsNumberInRange(*deltaYValue, INT_MIN);
            if (!deltaY || deltaY.value() != std::floor(deltaY.value())) {
                errorMessage = String("The 'deltaY' parameter for the action is invalid"_s);
                return std::nullopt;
            }
            action.deltaY = deltaY.value();
        } else {
            errorMessage = String("The 'deltaY' parameter for the action is missing"_s);
            return std::nullopt;
        }
        break;
    case Action::Subtype::KeyUp:
    case Action::Subtype::KeyDown:
    case Action::Subtype::PointerUp:
    case Action::Subtype::PointerDown:
    case Action::Subtype::PointerMove:
    case Action::Subtype::PointerCancel:
        ASSERT_NOT_REACHED();
    }

    return action;
}

static std::optional<PointerParameters> processPointerParameters(JSON::Object& actionSequence, std::optional<String>& errorMessage)
{
    PointerParameters parameters;

    auto parametersDataValue = actionSequence.getValue("parameters"_s);
    if (!parametersDataValue)
        return parameters;

    auto parametersData = parametersDataValue->asObject();
    if (!parametersData) {
        errorMessage = String("Action sequence pointer parameters is not an object"_s);
        return std::nullopt;
    }

    auto pointerType = parametersData->getString("pointerType"_s);
    if (!pointerType)
        return parameters;

    if (pointerType == "mouse"_s)
        parameters.pointerType = PointerType::Mouse;
    else if (pointerType == "pen"_s)
        parameters.pointerType = PointerType::Pen;
    else if (pointerType == "touch"_s)
        parameters.pointerType = PointerType::Touch;
    else {
        errorMessage = String("The 'pointerType' parameter in the pointer parameters of the action sequence is invalid"_s);
        return std::nullopt;
    }

    return parameters;
}

static std::optional<Vector<Action>> processInputActionSequence(Session& session, JSON::Value& actionSequenceValue, std::optional<String>& errorMessage)
{
    auto actionSequence = actionSequenceValue.asObject();
    if (!actionSequence) {
        errorMessage = String("The action sequence is not an object"_s);
        return std::nullopt;
    }

    auto type = actionSequence->getString("type"_s);
    InputSource::Type inputSourceType;
    if (type == "key"_s)
        inputSourceType = InputSource::Type::Key;
    else if (type == "pointer"_s)
        inputSourceType = InputSource::Type::Pointer;
    else if (type == "wheel"_s)
        inputSourceType = InputSource::Type::Wheel;
    else if (type == "none"_s)
        inputSourceType = InputSource::Type::None;
    else {
        errorMessage = String("The 'type' parameter in the action sequence is invalid or missing"_s);
        return std::nullopt;
    }

    auto id = actionSequence->getString("id"_s);
    if (!id) {
        errorMessage = String("The 'id' parameter in the action sequence is invalid or missing"_s);
        return std::nullopt;
    }

    std::optional<PointerParameters> parameters;
    std::optional<PointerType> pointerType;
    if (inputSourceType == InputSource::Type::Pointer) {
        parameters = processPointerParameters(*actionSequence, errorMessage);
        if (!parameters)
            return std::nullopt;

        pointerType = parameters->pointerType;
    }

    auto& inputSource = session.getOrCreateInputSource(id, inputSourceType, pointerType);
    if (inputSource.type != inputSourceType) {
        errorMessage = String("Action sequence type doesn't match input source type"_s);
        return std::nullopt;
    }

    if (inputSource.type ==  InputSource::Type::Pointer && inputSource.pointerType != pointerType) {
        errorMessage = String("Action sequence pointer type doesn't match input source pointer type"_s);
        return std::nullopt;
    }

    auto actionItems = actionSequence->getArray("actions"_s);
    if (!actionItems) {
        errorMessage = String("The 'actions' parameter in the action sequence is invalid or not present"_s);
        return std::nullopt;
    }

    Vector<Action> actions;
    unsigned actionItemsLength = actionItems->length();
    for (unsigned i = 0; i < actionItemsLength; ++i) {
        auto actionItem = actionItems->get(i)->asObject();
        if (!actionItem) {
            errorMessage = String("An action in the action sequence is not an object"_s);
            return std::nullopt;
        }

        std::optional<Action> action;
        if (inputSourceType == InputSource::Type::None)
            action = processNullAction(id, *actionItem, errorMessage);
        else if (inputSourceType == InputSource::Type::Key)
            action = processKeyAction(id, *actionItem, errorMessage);
        else if (inputSourceType == InputSource::Type::Pointer)
            action = processPointerAction(id, parameters.value(), *actionItem, errorMessage);
        else if (inputSourceType == InputSource::Type::Wheel)
            action = processWheelAction(id, *actionItem, errorMessage);
        if (!action)
            return std::nullopt;

        actions.append(action.value());
    }

    return actions;
}

void WebDriverService::performActions(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §17.5 Perform Actions.
    // https://w3c.github.io/webdriver/webdriver-spec.html#perform-actions
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto actionsArray = parameters->getArray("actions"_s);
    if (!actionsArray) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, String("The 'actions' parameter is invalid or not present"_s)));
        return;
    }

    std::optional<String> errorMessage;
    Vector<Vector<Action>> actionsByTick;
    unsigned actionsArrayLength = actionsArray->length();
    for (unsigned i = 0; i < actionsArrayLength; ++i) {
        auto actionSequence = actionsArray->get(i);
        auto inputSourceActions = processInputActionSequence(*m_session, actionSequence, errorMessage);
        if (!inputSourceActions) {
            completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, errorMessage.value_or("Could not process input action sequence"_s)));
            return;
        }
        for (unsigned i = 0; i < inputSourceActions->size(); ++i) {
            if (actionsByTick.size() < i + 1)
                actionsByTick.append({ });
            actionsByTick[i].append(inputSourceActions.value()[i]);
        }
    }

    m_session->performActions(WTFMove(actionsByTick), WTFMove(completionHandler));
}

void WebDriverService::releaseActions(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §17.5 Release Actions.
    // https://w3c.github.io/webdriver/webdriver-spec.html#release-actions
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->releaseActions(WTFMove(completionHandler));
}

void WebDriverService::dismissAlert(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §18.1 Dismiss Alert.
    // https://w3c.github.io/webdriver/webdriver-spec.html#dismiss-alert
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->dismissAlert(WTFMove(completionHandler));
    });
}

void WebDriverService::acceptAlert(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §18.2 Accept Alert.
    // https://w3c.github.io/webdriver/webdriver-spec.html#accept-alert
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->acceptAlert(WTFMove(completionHandler));
    });
}

void WebDriverService::getAlertText(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §18.3 Get Alert Text.
    // https://w3c.github.io/webdriver/webdriver-spec.html#get-alert-text
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->getAlertText(WTFMove(completionHandler));
    });
}

void WebDriverService::sendAlertText(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §18.4 Send Alert Text.
    // https://w3c.github.io/webdriver/webdriver-spec.html#send-alert-text
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto text = parameters->getString("text"_s);
    if (!text) {
        completionHandler(CommandResult::fail(CommandResult::ErrorCode::InvalidArgument, "Missing text parameter"_s));
        return;
    }

    m_session->waitForNavigationToComplete([this, text = WTFMove(text), completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->sendAlertText(text, WTFMove(completionHandler));
    });
}

void WebDriverService::takeScreenshot(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §19.1 Take Screenshot.
    // https://w3c.github.io/webdriver/webdriver-spec.html#take-screenshot
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    m_session->waitForNavigationToComplete([this, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->takeScreenshot(std::nullopt, std::nullopt, WTFMove(completionHandler));
    });
}

void WebDriverService::takeElementScreenshot(RefPtr<JSON::Object>&& parameters, Function<void (CommandResult&&)>&& completionHandler)
{
    // §19.2 Take Element Screenshot.
    // https://w3c.github.io/webdriver/webdriver-spec.html#take-element-screenshot
    if (!findSessionOrCompleteWithError(*parameters, completionHandler))
        return;

    auto elementID = findElementOrCompleteWithError(*parameters, completionHandler);
    if (!elementID)
        return;

    m_session->waitForNavigationToComplete([this, elementID, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            completionHandler(WTFMove(result));
            return;
        }
        m_session->takeScreenshot(elementID.value(), true, WTFMove(completionHandler));
    });
}

#if ENABLE(WEBDRIVER_BIDI)
void WebDriverService::bidiSessionStatus(unsigned id, RefPtr<JSON::Object>&&, Function<void(WebSocketMessageHandler::Message&&)>&& completionHandler)
{
    auto result = JSON::Object::create();
    bool ready = !m_session;
    result->setBoolean("ready"_s, ready);
    if (ready)
        result->setString("message"_s, "Ready for new sessions"_s);
    else
        result->setString("message"_s, "Maximum number of sessions created"_s);

    completionHandler(WebSocketMessageHandler::Message::reply("success"_s, id, WTFMove(result)));
}

void WebDriverService::bidiSessionSubscribe(unsigned id, RefPtr<JSON::Object>&&parameters, Function<void(WebSocketMessageHandler::Message&&)>&& completionHandler)
{
    // https://w3c.github.io/webdriver-bidi/#command-session-subscribe
    auto eventNamesJSON = parameters->getArray("events"_s);

    if (!eventNamesJSON) {
        completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, std::nullopt, "Missing 'events' parameter"_s, id));
        return;
    }

    // FIXME: Support event priorities.
    // https://bugs.webkit.org/show_bug.cgi?id=282436
    // FIXME: Support by-context subscriptions.
    // https://bugs.webkit.org/show_bug.cgi?id=282981
    Vector<String> eventNames;
    for (auto& eventNameJS : *eventNamesJSON) {
        auto eventName = eventNameJS->asString();
        if (!eventName) {
            completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, std::nullopt, "Invalid event name"_s, id));
            return;
        }
        eventNames.append(eventName);
    }
    m_session->subscribeForEvents(eventNames, { }, { }, [id, completionHandler = WTFMove(completionHandler)](CommandResult&& subscriptionResult) mutable {
        if (subscriptionResult.isError()) {
            auto errorMessage = subscriptionResult.errorMessage() ? subscriptionResult.errorMessage() : "Failed to subscribe"_s;
            completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, std::nullopt, errorMessage, id));
            return;
        }
        auto result = JSON::Object::create();
        result->setString("subscription"_s, subscriptionResult.result()->asString());

        completionHandler(WebSocketMessageHandler::Message::reply("success"_s, id, WTFMove(result)));
    });
}

void WebDriverService::bidiSessionUnsubscribe(unsigned id, RefPtr<JSON::Object>&&parameters, Function<void(WebSocketMessageHandler::Message&&)>&& completionHandler)
{
    // https://w3c.github.io/webdriver-bidi/#command-session-unsubscribe
    auto subscriptions = parameters->getArray("subscriptions"_s);
    if (!subscriptions) {
        auto events = parameters->getArray("events"_s);
        if (!events) {
            completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, std::nullopt, "Missing either 'events' or 'subscriptions' parameter"_s, id));
            return;
        }

        Vector<String> eventNames;
        for (auto& event : *events) {
            auto eventName = event->asString();
            if (!eventName) {
                completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, std::nullopt, "Invalid event name"_s, id));
                return;
            }

            eventNames.append(eventName);
        }

        m_session->unsubscribeByEventName(eventNames, [id, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
            if (result.isError()) {
                auto errorMessage = result.errorMessage() ? result.errorMessage() : "Failed to unsubscribe"_s;
                completionHandler(WebSocketMessageHandler::Message::fail(result.errorCode(), std::nullopt, errorMessage, id));
                return;
            }
            completionHandler(WebSocketMessageHandler::Message::reply("success"_s, id, JSON::Value::null()));
        });
        return;
    }

    Vector<String> subscriptionsIDs;
    for (auto& subscription : *subscriptions) {
        auto subscriptionID = subscription->asString();
        if (!subscriptionID) {
            completionHandler(WebSocketMessageHandler::Message::fail(CommandResult::ErrorCode::InvalidArgument, std::nullopt, "Invalid subscription ID"_s, id));
            return;
        }
        subscriptionsIDs.append(subscriptionID);
    }

    m_session->unsubscribeByIDs(WTFMove(subscriptionsIDs), [id, completionHandler = WTFMove(completionHandler)](CommandResult&& result) mutable {
        if (result.isError()) {
            auto errorMessage = result.errorMessage() ? result.errorMessage() : "Failed to unsubscribe"_s;
            completionHandler(WebSocketMessageHandler::Message::fail(result.errorCode(), std::nullopt, errorMessage, id));
            return;
        }
        completionHandler(WebSocketMessageHandler::Message::reply("success"_s, id, JSON::Value::null()));
    });
}

void WebDriverService::clientDisconnected(const WebSocketMessageHandler::Connection& connection)
{
    // https://w3c.github.io/webdriver-bidi/#handle-a-connection-closing
    if (m_bidiServer->sessionID(connection) == m_session->id())
        m_bidiServer->removeConnection(connection);
    else if (m_bidiServer->isStaticConnection(connection))
        m_bidiServer->removeStaticConnection(connection);
    // Note from spec: This does not end any session.
}

void WebDriverService::onBrowserTerminated(const String& sessionID)
{
    if (m_session && m_session->id() == sessionID) {
        auto connection = m_bidiServer->connection(sessionID);
        m_bidiServer->disconnectSession(sessionID);
        if (connection)
            clientDisconnected(*connection);
    }
}

#endif // ENABLE(WEBDRIVER_BIDI)

} // namespace WebDriver
