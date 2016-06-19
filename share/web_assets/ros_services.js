/*
 * Copyright 2012 the original author or authors.
 * See the NOTICE file distributed with this work for additional
 * information regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
class RosTopic
{
    static get parameters()
    {
        return [RosLib];
    }

    constructor(ros)
    {
        this.ros = ros.getRos();
    }

    getTopic(topic, type)
    {
        var topic = new ROSLIB.Topic({
            ros: this.ros,
            name: topic,
            messageType: type
        });

        var subscribers = [];
        return Rx.Observable.create(observer => {
            subscribers.push(observer);
            if (subscribers.length == 1) {
                topic.subscribe(message => {
                    for (let subscriber of subscribers) {
                        subscriber.next(message);
                    }
                });
            }
            return () => {
                var index = subscribers.findIndex(value => value == observer);
                subscribers[index] = subscribers[subscribers.length];
                subscribers.pop();
                if (subscribers.length == 0) {
                    topic.unsubscribe();
                }
            };
        });
    }
}

class RosParam
{
    static get parameters()
    {
        return [RosLib];
    }

    constructor(ros)
    {
        this.ros = ros.getRos();
    }

    get(key)
    {
        var param = new ROSLIB.Param({ros: this.ros, name: key});
        return Rx.Observable.create(observer => {
            param.get(value => observer.next(value));
        });
    }

    set(key, value)
    {
        var param = new ROSLIB.Param({ros: this.ros, name: key});
        param.set(value);
    }
}

class RosService
{
    static get parameters()
    {
        return [RosLib];
    }

    constructor(ros)
    {
        this.ros = ros.getRos();
    }

    getService(service_name, type)
    {
        var service = new ROSLIB.Service({
            ros: this.ros,
            name: service_name,
            messageType: type
        });

        return (args, callback) => {
            if (typeof args === 'undefined') {
                args = {};
            }
            if (typeof callback === 'undefined') {
                callback = function(result) {};
            }
            service.callService(new ROSLIB.ServiceRequest(args), callback);
        };
    }
}

class RosLib
{
    constructor()
    {
        var url = 'ws://' + window.location.hostname + ':8088';
        this.ros = new ROSLIB.Ros({url: url});
    }

    getRos()
    {
        return this.ros;
    }
}
