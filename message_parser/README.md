# Message Parser

##

```mermaid
graph LR
    T1[Simulator] -- /message --> Node((message_parser))


    Node -- /message/person -->D[global manager]
    Node -- /message/object -->D[global manager]
    Node -- /message/object_num -->D[global manager]
    Node -- /message/object_darknet -->D[global manager]
```


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

