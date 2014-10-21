  while(true){
    sTime = CurrentTick();
    OnFwd(OUT_BC, 30);
    int color = getColor();
    if ( (color != BLK) && (color != YEL) ){
      Off(OUT_BC);
      t1 = CurrentTick() - sTime;
    
      //reverse
      moveRev(30);
      if(getColor() == BLK && getColor() == YEL);
      Off(OUT_BC);
   
	    if(t1 > t0){
	      d1 = d0
	    }
	      else if(t1 <t0){
	      d1 = //opposite?
	    }
	       
	      //turn bot
	    if(d1 == 0){
	      OnFwd(OUT_C, speed);
	      OnRev(OUT_B, speed);
	    }
	    else{
	      OnFwd(OUT_B, speed);
	      OnRev(OUT_C, speed);
	    }
  	}
	//t0 = CurrentTick() - sTime;

	//moveRev(30);
	//until(getColor() == BLK && getColor() == YEL);