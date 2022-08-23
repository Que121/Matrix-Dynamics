
var odiv1=document.getElementsByClassName("box1")[0];
		var odiv2=document.getElementsByClassName("box2")[0];

		window.onkeydown=function(e){通过监听按键按下时的操作
			var e=e||window.event;
			//alert(e.keyCode)
            //获取当前元素到body的距离 
			var T=odiv2.offsetTop//盒子距离最顶部的距离
			var L=odiv2.offsetLeft/盒子距离最左边的距离
			if (e.keyCode==87) {//判断当前按键是不是w按键
				var T=T-5
				if(T<0){
					T=0
				}
				odiv2.style.top=T+"px"
			}
			if (e.keyCode==83) {//判断当前按键是不是S按键
				var T=T+5
				if (T>odiv1.offsetWidth-odiv2.offsetWidth) {
					T=odiv1.offsetWidth-odiv2.offsetWidth					
				}
				odiv2.style.top=T+"px"
			}
			if (e.keyCode==65) {//判断当前按键是不是A按键
				var L=L-5
				if(L<0){
					L=0
				}
				odiv2.style.left=L+"px"
			}
			if (e.keyCode==68) {//判断当前按键是不是D按键
				var L=L+5
				if(L>odiv1.offsetHeight-odiv2.offsetHeight){
					L=odiv1.offsetHeight-odiv2.offsetHeight
				}
				odiv2.style.left=L+"px"
			}	
}
