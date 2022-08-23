function forward() {
    $("#forward").on("click",function (event) {
        $.ajax({
            url:"/user/forward",
            method:'POST'
        })
    })

}

$(function () {
    forward();
})