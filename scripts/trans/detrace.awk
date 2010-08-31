#! /usr/bin/awk -f

BEGIN { 
#  FS="[[:space:]+:]"  
  FS="[ :]+";
    activ=0; noTime=1; iterTime=-1;
  for( i=0;i<ARGC;i++) 
    {
      pattern="iter=";
      if( substr(ARGV[i],1,5)==pattern ) 
      {
	iterTime=substr(ARGV[i],6,10); active=2; noTime=0; 
      }
    }
  print "iter=" iterTime ";";
}

(noTime==0) && /--- Time [0-9]* ----/   {
  for (idx=0;;idx++)
  {    if( $idx=="Time" ) { idx++; break; }  }
  if( $idx == iterTime)  active=0; 
  else if( active==0) exit; 
}

!/\]/ &&(active==1) { 
  print;
  }

/\]/ && (active==1) {
  active=0;
  print $0 ";" ;
}

/[A-Za-z].* *= *\[/ && !/\]/ &&(active==0) {
  for (idx=0;;idx++)
  {
    if( substr($idx,1,1)=="[" ) 
    {
       startv=idx;
       for(idx--;$idx!="=";idx--);
       for(idx--;$idx=="";idx--);
       if( $startv!="[" ) { $startv=substr($startv,2,length($startv)); startv--; }
      break;
    }
  }
  print $idx " = [ ";
  for (i=startv+1;i<NF;i++)
  {    
    printf(" %s",$i);
  }
  print "";
  active = 1;
}

/[A-Za-z].* *= *\[/ && /\]/  &&(active==0) {
  for (idx=0;;idx++)
  {    if( substr($idx,1,1)=="[" ) 
     {
       startv=idx;
       for(idx--;$idx!="=";idx--);
       for(idx--;$idx=="";idx--);
       if( $startv!="[" ) { $startv=substr($startv,2,length($startv)); startv--; }
       break; 
     }
  }
  printf("%s = [",$idx);
  for (i=startv+1;i<=NF;i++)
  {    
   printf(" %s",$i);
  }
  print "'; " ;
}

