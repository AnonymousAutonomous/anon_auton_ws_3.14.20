function sendViaAntenna(event) {
    console.log(event);
    event.preventDefault();

    console.log(event);

    //get the action-url of the form
    var actionurl = event.currentTarget.action;
    console.log(actionurl)

    //do your own request an handle the results
    $.ajax({
            url: actionurl,
            type: 'post',
            dataType: 'application/json',
            data: $("#myform").serialize(),
            success: function(data) {
                console.log(data)
            }
    });

    setTimeout( () => {
        console.log(event);
    }, 5000)
    
}
