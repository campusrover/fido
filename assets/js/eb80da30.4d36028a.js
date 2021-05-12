(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[1128],{3905:function(e,t,n){"use strict";n.d(t,{Zo:function(){return s},kt:function(){return d}});var r=n(7294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function i(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function a(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?i(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):i(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,r,o=function(e,t){if(null==e)return{};var n,r,o={},i=Object.keys(e);for(r=0;r<i.length;r++)n=i[r],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(r=0;r<i.length;r++)n=i[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var c=r.createContext({}),p=function(e){var t=r.useContext(c),n=t;return e&&(n="function"==typeof e?e(t):a(a({},t),e)),n},s=function(e){var t=p(e.components);return r.createElement(c.Provider,{value:t},e.children)},u={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},f=r.forwardRef((function(e,t){var n=e.components,o=e.mdxType,i=e.originalType,c=e.parentName,s=l(e,["components","mdxType","originalType","parentName"]),f=p(n),d=o,m=f["".concat(c,".").concat(d)]||f[d]||u[d]||i;return n?r.createElement(m,a(a({ref:t},s),{},{components:n})):r.createElement(m,a({ref:t},s))}));function d(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var i=n.length,a=new Array(i);a[0]=f;var l={};for(var c in t)hasOwnProperty.call(t,c)&&(l[c]=t[c]);l.originalType=e,l.mdxType="string"==typeof e?e:o,a[1]=l;for(var p=2;p<i;p++)a[p]=n[p];return r.createElement.apply(null,a)}return r.createElement.apply(null,n)}f.displayName="MDXCreateElement"},8710:function(e,t,n){"use strict";n.r(t),n.d(t,{frontMatter:function(){return a},metadata:function(){return l},toc:function(){return c},default:function(){return s}});var r=n(2122),o=n(9756),i=(n(7294),n(3905)),a={sidebar_label:"config",title:"fido.config"},l={unversionedId:"reference/fido/config",id:"reference/fido/config",isDocsHomePage:!1,title:"fido.config",description:"set\\docker\\host",source:"@site/docs/reference/fido/config.md",sourceDirName:"reference/fido",slug:"/reference/fido/config",permalink:"/fido/docs/reference/fido/config",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/reference/fido/config.md",version:"current",sidebar_label:"config",frontMatter:{sidebar_label:"config",title:"fido.config"},sidebar:"apiSidebar",previous:{title:"fido.world.world",permalink:"/fido/docs/reference/fido/world/world"},next:{title:"fido.core",permalink:"/fido/docs/reference/fido/core"}},c=[],p={toc:c};function s(e){var t=e.components,n=(0,o.Z)(e,["components"]);return(0,i.kt)("wrapper",(0,r.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h4",{id:"set_docker_host"},"set","_","docker","_","host"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},'set_docker_host(base_url="tcp://127.0.0.1:1234", version="1.35")\n')),(0,i.kt)("p",null,"Set the Docker client connection details."),(0,i.kt)("p",null,(0,i.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"base_url")," ",(0,i.kt)("em",{parentName:"li"},"str")," - URL to Docker server. For example,\n",(0,i.kt)("inlineCode",{parentName:"li"},"unix:///var/run/docker.sock")," or ",(0,i.kt)("inlineCode",{parentName:"li"},"tcp://127.0.0.1:1234"),". Default:\n",(0,i.kt)("inlineCode",{parentName:"li"},"tcp://127.0.0.1:1234"),"."),(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"version")," ",(0,i.kt)("em",{parentName:"li"},"str")," - The version of the API to use. Set to ",(0,i.kt)("inlineCode",{parentName:"li"},"auto")," to\nautomatically detect the server","'","s version. Default: ",(0,i.kt)("inlineCode",{parentName:"li"},"1.35"),".")),(0,i.kt)("p",null,(0,i.kt)("strong",{parentName:"p"},"Raises"),":"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"fido.errors.DockerError")," - If the specified Docker server does not exist,\nor failed to connect.")),(0,i.kt)("h4",{id:"set_logging"},"set","_","logging"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},"set_logging(node_name, level)\n")),(0,i.kt)("p",null,"Enable logging for a given node, and its logging level."),(0,i.kt)("p",null,"This is a legacy feature inherited from ",(0,i.kt)("inlineCode",{parentName:"p"},"robot_services"),". See\n",(0,i.kt)("inlineCode",{parentName:"p"},"robot_services"),"'","s documentation for more details."),(0,i.kt)("p",null,(0,i.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"node_name")," ",(0,i.kt)("em",{parentName:"li"},"str")," - The name of the node."),(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"level")," ",(0,i.kt)("em",{parentName:"li"},"str")," - Description of the log","'","s type.")))}s.isMDXComponent=!0}}]);