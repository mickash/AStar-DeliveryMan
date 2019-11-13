#' aStarDM
#'
#' The par function for the delivery man project.
#' @param roads See runDeliveryMan function in DeliveryMan project for details
#' @param car See runDeliveryMan function in DeliveryMan project for details
#' @param packages See runDeliveryMan function in DeliveryMan project for details
#' @return packages See runDeliveryMan function in DeliveryMan project for details
#' @export
aStarDM=function(roads,car,packages){
  x_goal=NULL
  y_goal=NULL
  if (car$load==0){
    # Find geometrically closest next package
    available=(1:5)[which(packages[,5]==0)]
    dists=sapply(available,function(i){
      abs(car$x-packages[i,1])+abs(car$y-packages[i,2])
    })
    chosen=available[which.min(dists)]
    x_goal=packages[chosen,1]
    y_goal=packages[chosen,2]
  }
  else {
    x_goal=packages[car$load,3]
    y_goal=packages[car$load,4]
  }
  h=makeAStarHeuristic(x_goal,y_goal,roads$hroads,roads$vroads)
  car$nextMove=aStar(car$x,car$y,x_goal,y_goal,roads$hroads,roads$vroads,h)
  return (car)
}
#' @keywords internal
makeAStarHeuristic=function(x_goal,y_goal,vroads,hroads) {
  v_min=min(vroads)
  h_min=min(hroads)
  function(x,y){
    abs(x-x_goal)*h_min+abs(y-y_goal)*v_min
  }
}
#' @keywords internal
aStarNode=function(x,y,g,h,path) {
  # x,y : position
  # g: cost to go to
  # h: heuristic
  # f: total cost = g+h
  # path: route to node
  list(x=x,y=y,f=g+h,g=g,h=h,path=path)
}
#' @keywords internal
makeNode=function(node,x_dif,y_dif,cost_dif,h,move){
  # We place node in a list for use in append
  list(aStarNode(x=node$x+x_dif,
                 y=node$y+y_dif,
                 g=node$g+cost_dif,
                 h=h,
                 path=c(node$path,move)))
}
#' @keywords internal
getNeighbors=function(node,vroads,hroads,h){
  out=list()
  if (node$y<=ncol(vroads)){
    out=append(out,makeNode(node,0,1,vroads[node$x,node$y],h(node$x,node$y+1),8))
  }
  if (node$x<=nrow(hroads)){
    out=append(out,makeNode(node,1,0,hroads[node$x,node$y],h(node$x+1,node$y),6))
  }
  if (node$x>1){
    out=append(out,makeNode(node,-1,0,hroads[node$x-1,node$y],h(node$x-1,node$y),4))
  }
  if (node$y>1){
    out=append(out,makeNode(node,0,-1,vroads[node$x,node$y-1],h(node$x,node$y-1),2))
  }
  return (out)
}
#' @keywords internal
aStar=function(x,y,x_goal,y_goal,hroads,vroads,h){
  start=aStarNode(x,y,0,h(x,y),c())
  frontier=list(start)
  out=5
  repeat {
    nextNode=frontier[[1]]
    if (nextNode$x==x_goal && nextNode$y==y_goal) {
      if (length(nextNode$path)==0)
        out=5
      else
        out=nextNode$path[1]
      break
    }
    # Remove next node from frontier
    frontier=frontier[-1]
    # Find neighbors
    neighbors=getNeighbors(nextNode,vroads,hroads,h)
    # Add neighbors to frontier
    newNodes=rep(T,length(neighbors))
    if (length(frontier)==0)
      frontier=neighbors
    else {
      for (n in 1:length(neighbors)){
        for (i in 1:length(frontier)) {
          if (neighbors[[n]]$x==frontier[[i]]$x && neighbors[[n]]$y==frontier[[i]]$y){
            if (neighbors[[n]]$f < frontier[[i]]$f)
              frontier[[i]]=neighbors[[n]]
            newNodes[n]=F
            break
          }
        }
      }
      if (any(newNodes))
        frontier=append(frontier,neighbors[newNodes])
    }
    # Reorder frontier
    costs=sapply(frontier,function(n)n$f)
    frontier=frontier[order(costs)]
  }
  return (out)
}
