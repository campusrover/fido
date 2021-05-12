(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[6072],{3905:function(e,t,n){"use strict";n.d(t,{Zo:function(){return d},kt:function(){return m}});var r=n(7294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function a(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function l(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?a(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):a(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function i(e,t){if(null==e)return{};var n,r,o=function(e,t){if(null==e)return{};var n,r,o={},a=Object.keys(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var p=r.createContext({}),s=function(e){var t=r.useContext(p),n=t;return e&&(n="function"==typeof e?e(t):l(l({},t),e)),n},d=function(e){var t=s(e.components);return r.createElement(p.Provider,{value:t},e.children)},u={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},c=r.forwardRef((function(e,t){var n=e.components,o=e.mdxType,a=e.originalType,p=e.parentName,d=i(e,["components","mdxType","originalType","parentName"]),c=s(n),m=o,k=c["".concat(p,".").concat(m)]||c[m]||u[m]||a;return n?r.createElement(k,l(l({ref:t},d),{},{components:n})):r.createElement(k,l({ref:t},d))}));function m(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var a=n.length,l=new Array(a);l[0]=c;var i={};for(var p in t)hasOwnProperty.call(t,p)&&(i[p]=t[p]);i.originalType=e,i.mdxType="string"==typeof e?e:o,l[1]=i;for(var s=2;s<a;s++)l[s]=n[s];return r.createElement.apply(null,l)}return r.createElement.apply(null,n)}c.displayName="MDXCreateElement"},7617:function(e,t,n){"use strict";n.r(t),n.d(t,{frontMatter:function(){return l},metadata:function(){return i},toc:function(){return p},default:function(){return d}});var r=n(2122),o=n(9756),a=(n(7294),n(3905)),l={sidebar_label:"robot",title:"fido.robot.robot"},i={unversionedId:"reference/fido/robot/robot",id:"reference/fido/robot/robot",isDocsHomePage:!1,title:"fido.robot.robot",description:"Robot",source:"@site/docs/reference/fido/robot/robot.md",sourceDirName:"reference/fido/robot",slug:"/reference/fido/robot/robot",permalink:"/fido/docs/reference/fido/robot/robot",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/reference/fido/robot/robot.md",version:"current",sidebar_label:"robot",frontMatter:{sidebar_label:"robot",title:"fido.robot.robot"},sidebar:"apiSidebar",previous:{title:"fido.robot.component.sensor",permalink:"/fido/docs/reference/fido/robot/component/sensor"},next:{title:"fido.robot.turtlebot3",permalink:"/fido/docs/reference/fido/robot/turtlebot3"}},p=[{value:"Robot",id:"robot",children:[{value:"add_sensor",id:"add_sensor",children:[]},{value:"prepare",id:"prepare",children:[]},{value:"connect",id:"connect",children:[]},{value:"ros",id:"ros",children:[]},{value:"set_world",id:"set_world",children:[]},{value:"move",id:"move",children:[]},{value:"rotate",id:"rotate",children:[]},{value:"stop",id:"stop",children:[]},{value:"ros_robot_description",id:"ros_robot_description",children:[]},{value:"ros_fill_dependency",id:"ros_fill_dependency",children:[]}]}],s={toc:p};function d(e){var t=e.components,n=(0,o.Z)(e,["components"]);return(0,a.kt)("wrapper",(0,r.Z)({},s,n,{components:t,mdxType:"MDXLayout"}),(0,a.kt)("h2",{id:"robot"},"Robot"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class Robot(ABC,  RobotProtocol)\n")),(0,a.kt)("p",null,"Represents a physical or simulated robot."),(0,a.kt)("h3",{id:"add_sensor"},"add","_","sensor"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},' | def add_sensor(sensor_cls: Type["Sensor"], sensor_args: Mapping[str, Any] = {}) -> None\n')),(0,a.kt)("p",null,"Add sensor to the robot."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"sensor_cls")," ",(0,a.kt)("em",{parentName:"li"},"Type","[Sensor]")," - Sensor class."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"sensor_args")," ",(0,a.kt)("em",{parentName:"li"},"dict")," - Arguments mapping for initializing sensor.")),(0,a.kt)("h3",{id:"prepare"},"prepare"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | def prepare() -> None\n")),(0,a.kt)("p",null,"Prepare initializes all the sensors in the robot."),(0,a.kt)("h3",{id:"connect"},"connect"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | def connect(host: str, port: int) -> None\n")),(0,a.kt)("p",null,"Connect to the robot via ROS bridge."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"host")," ",(0,a.kt)("em",{parentName:"li"},"str")," - Name or IP address of the ROS bridge host, e.g. ",(0,a.kt)("inlineCode",{parentName:"li"},"127.0.0.1"),"."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"port")," ",(0,a.kt)("em",{parentName:"li"},"int")," - ROS bridge port, e.g. ",(0,a.kt)("inlineCode",{parentName:"li"},"9090"),".")),(0,a.kt)("h3",{id:"ros"},"ros"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | def ros() -> Ros\n")),(0,a.kt)("p",null,"Return internal ROS client."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Returns"),":"),(0,a.kt)("p",null,"  The ROS client."),(0,a.kt)("h3",{id:"set_world"},"set","_","world"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},' | def set_world(world: "World") -> None\n')),(0,a.kt)("p",null,"Set the world to use for this robot."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"world")," ",(0,a.kt)("em",{parentName:"li"},"World")," - Parent world.")),(0,a.kt)("h3",{id:"move"},"move"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | @abstractmethod\n | def move(distance: float = 0, duration: float = 0, speed: float = 0) -> None\n")),(0,a.kt)("p",null,"Move the robot at a certain distance at a certain speed or for a\ncertain duration."),(0,a.kt)("p",null,"To move backwards, set speed to negative. If the given speed is\nlarger than the maximum speed, it will be set to the maximum\nspeed."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"distance")," ",(0,a.kt)("em",{parentName:"li"},"float")," - Distance to travel."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"duration")," ",(0,a.kt)("em",{parentName:"li"},"float")," - Time duration to travel for (in seconds)."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"speed")," ",(0,a.kt)("em",{parentName:"li"},"float")," - Travel speed.")),(0,a.kt)("h3",{id:"rotate"},"rotate"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | @abstractmethod\n | def rotate(angle: float = 0, duration: float = 0, speed: float = 0) -> None\n")),(0,a.kt)("p",null,"Rotate the robot at a certain angle at a certain speed or for a\ncertain duration."),(0,a.kt)("p",null,"To rotate clockwise, set the speed to positive. To rotate in\nanti-clockwise, set the speed to negative. If the given speed is\nlarger  than the maximum speed, it will be set to the maximum\nspeed."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"angle")," ",(0,a.kt)("em",{parentName:"li"},"float")," - Angle to rotate (in degrees)."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"duration")," ",(0,a.kt)("em",{parentName:"li"},"float")," - Time duration to rotate for (in seconds)."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"speed")," ",(0,a.kt)("em",{parentName:"li"},"float")," - Rotation speed (radian per seconds).")),(0,a.kt)("h3",{id:"stop"},"stop"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | @abstractmethod\n | def stop(forced: bool = False) -> None\n")),(0,a.kt)("p",null,"Stop the robot."),(0,a.kt)("p",null,"This is a blocking call. It will block execution until the robot\nis gracefully stopped unless ",(0,a.kt)("inlineCode",{parentName:"p"},"forced")," is set to ",(0,a.kt)("inlineCode",{parentName:"p"},"True"),"."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"forced")," ",(0,a.kt)("em",{parentName:"li"},"bool")," - Forcefully stop the robot or not.")),(0,a.kt)("h3",{id:"ros_robot_description"},"ros","_","robot","_","description"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | @abstractmethod\n | def ros_robot_description() -> str\n")),(0,a.kt)("p",null,"Return the ROS specific robot description."),(0,a.kt)("p",null,"This is mainly used for building the launch file. E.g.\n",(0,a.kt)("inlineCode",{parentName:"p"},"robot_description/urdf/model.urdf"),"."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Returns"),":"),(0,a.kt)("p",null,"  The robot_description used by the launch file."),(0,a.kt)("h3",{id:"ros_fill_dependency"},"ros","_","fill","_","dependency"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},' | @abstractmethod\n | def ros_fill_dependency(installfile: "InstallFile") -> None\n')),(0,a.kt)("p",null,"Fill the needed dependencies to the given installfile."),(0,a.kt)("p",null,"E.g. ",(0,a.kt)("inlineCode",{parentName:"p"},"installfile.git(\n&quot;src/turtlebot3&quot;,\n&quot;https://github.com/ROBOTIS-GIT/turtlebot3.git&quot;,\n&quot;master&quot;,\n)")),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"installfile")," ",(0,a.kt)("em",{parentName:"li"},"InstallFile")," - InstallFile for filling dependencies.")))}d.isMDXComponent=!0}}]);