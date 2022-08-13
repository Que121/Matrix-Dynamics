function bindCaptchaBtnClick() {
    $("#captcha-btn").on("click",function (event) {
        // 括号外加上$代表jquery对象$("#captcha-btn")
        var $this = $(this);
        var email = $("input[name='email']").val();
        if(!email){
            alert("请先输入邮箱！");
            return;
        }
        // 通过js发送网络请求：ajax
        $.ajax({
            url:"/user/captcha",
            method:"POST",
            data:{
                "email":email
            },
            success: function (res) {
                var code = res['code'];
                if(code == 200){
                    // 取消点击事件
                    $this.off("click");
                    // 开始倒计时
                    var countDown = 60;
                    // javascript内置的定时器
                    // 每隔1000毫秒修改一次
                    var timer = setInterval(function(){
                        countDown -= 1;
                        if(countDown>0){
                            $this.text(countDown+"秒后重新发送");
                        }else{
                            $this.text("获取验证码");
                            // 重新执行一下这个函数，重新执行这个绑定事件
                            bindCaptchaBtnClick();
                            // clearInterval清除定时器
                            // 如果不需要倒计时了。那么就要记得清除倒计时，否则会一直执行下去
                            clearInterval(timer);
                        }
                    },1000);
                    alert("验证码发送成功！");
                }else {
                    alert(res['message']);
                }
            }
        })
    });
}

// 带$代表在文档加载完之后才会执行
$(function () {
    bindCaptchaBtnClick();
});