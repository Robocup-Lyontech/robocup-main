# Message Parser

## Topics

![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFJcbiAgICBUMVtTaW11bGF0b3JdIC0tIC9tZXNzYWdlIC0tPiBOb2RlKChtZXNzYWdlX3BhcnNlcikpXG5cblxuICAgIE5vZGUgLS0gL21lc3NhZ2UvcGVyc29uIC0tPkRbZ2xvYmFsIG1hbmFnZXJdXG4gICAgTm9kZSAtLSAvbWVzc2FnZS9vYmplY3QgLS0-RFtnbG9iYWwgbWFuYWdlcl1cbiAgICBOb2RlIC0tIC9tZXNzYWdlL29iamVjdF9udW0gLS0-RFtnbG9iYWwgbWFuYWdlcl1cbiAgICBOb2RlIC0tIC9tZXNzYWdlL29iamVjdF9kYXJrbmV0IC0tPkRbZ2xvYmFsIG1hbmFnZXJdXG4iLCJtZXJtYWlkIjoie1xuICBcInRoZW1lXCI6IFwiZGVmYXVsdFwiXG59IiwidXBkYXRlRWRpdG9yIjp0cnVlLCJhdXRvU3luYyI6dHJ1ZSwidXBkYXRlRGlhZ3JhbSI6dHJ1ZX0)

`/message/person` possible values :  
- "pending"
- "left"
- "right"
- "undefined"

`/message/object` possible values :  
- "pending"
- label given by /message and matching with the ycb dataset
- "undefined"

`/message/object_num` possible values :  
- -1 :  pending
- 0 : undefined
- 1-77 : ycb object number  

`/message/object_darknet` possible values :  
- "pending"
- label from darknet classes
- "undefined"

If you receive "pending" or -1, wait for receiving /message  


## Service

`message_parser` with following feedback (same types/contents as above):    
- `person`  
- `object`  
- `object_num`  
- `object_darknet`

