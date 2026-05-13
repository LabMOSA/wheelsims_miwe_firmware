void AdjContact()
{
  //////////////////////////////
  //// Contact scenarios
  ///////////////////////////////

  int bumpval = 6;
  
    if (ContactY < 0.05 ) {
      // Can't Reverse, bump both wheels forward
      adjCont[0]  = int( - bumpval);
      adjCont[1]  = int( - bumpval);
    }
    else if (ContactY > 0.95) {
      //Can't go forward, bump both wheels back
      adjCont[0]  = int(  bumpval);
      adjCont[1]  = int(  bumpval);
    }
    else if (ContactX < 0.25 && ContactY > 0.5)  {
      //Contact front Left, bump right wheel back
      adjCont[0]  = int(0);
      adjCont[1]  = int( bumpval / 2);
    }
    else if (ContactX > 0.75  && ContactY > 0.5) {
      //Contact front Right, bump left wheel back
      adjCont[0]  = int( bumpval / 2);
      adjCont[1]  = int(0);
    }
    else if (ContactX > 0.75 && ContactY < 0.5) {
      //Contact back Left, bump right wheel forward
      adjCont[0]  = int(0 );
      adjCont[1]  = int( - bumpval / 2);
    }
    else if (ContactX < 0.25 && ContactY < 0.5) {
      //Contact back right, bump left wheel forward
      adjCont[0]  = int(- bumpval / 2);
      adjCont[1] = int(0) ;
    }
  
}
