"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g;
    return g = { next: verb(0), "throw": verb(1), "return": verb(2) }, typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (_) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
var rclnodejs = require("rclnodejs");
var utils_1 = require("./utils");
var NODE_NAME = "TemperatureGenNode";
var INTERVAL_MS = 1000;
var TOPIC = "temperature";
var FRAME_ID = "Temperature";
var LOGGER_NAME = "TEMPGEN";
var TEMP = 100;
var TEMP_STD_DEV = 10;
var TemperatureNode = /** @class */ (function () {
    function TemperatureNode() {
    }
    // create node and publisher
    TemperatureNode.prototype.init = function () {
        return __awaiter(this, void 0, void 0, function () {
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0: return [4 /*yield*/, rclnodejs.init()];
                    case 1:
                        _a.sent();
                        this.node = rclnodejs.createNode(NODE_NAME);
                        this.logger = rclnodejs.logging.getLogger(LOGGER_NAME);
                        this.logger.info('Starting ' + NODE_NAME);
                        this.publisher = this.node.createPublisher('sensor_msgs/msg/Temperature', TOPIC);
                        this.clock = new rclnodejs.ROSClock();
                        return [2 /*return*/];
                }
            });
        });
    };
    // generate temperature msgs every second
    TemperatureNode.prototype.start = function () {
        if (this.timer && !this.timer.isCanceled()) {
            this.logger.error(NODE_NAME + ' already started.');
            return;
        }
        else {
            this.node.destroyTimer(this.timer);
        }
        this.node.createTimer(INTERVAL_MS, this.createAndPublishTemperature);
        rclnodejs.spin(this.node);
    };
    TemperatureNode.prototype.stop = function () {
        if (this.timer.isCanceled())
            return;
        this.timer.cancel();
    };
    TemperatureNode.prototype.createAndPublishTemperature = function () {
        // create random temp
        var temp = TEMP + TEMP_STD_DEV - 2 * TEMP_STD_DEV * Math.random();
        // create ros2 temperature msg
        var msg = rclnodejs.createMessageObject('sensor_msgs/msg/LaserScan');
        // init msg header
        msg.header.frame_id = FRAME_ID;
        msg.header.stamp = utils_1.ROSUtils.createMsgHeaderTimestamp(this.clock.now());
        msg.temperature = temp;
        msg.variance = 0.0;
        // publish temperature msg
        this.publisher.publish(msg);
    };
    return TemperatureNode;
}());
exports.TemperatureNode = TemperatureNode;
//# sourceMappingURL=temperature_node.js.map