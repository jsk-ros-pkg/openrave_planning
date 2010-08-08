% links = orBodyGetLinks(bodyid)
%
% Returns the transformations of all the body's links in a 12 x L matrix. Where L
% is the number of links and each column is a 3x4 transformation
% (use T=reshape(., [3 4]) to recover).
% T * [X;1] = Xnew

function links = orBodyGetLinks(bodyid)

%% get some body info
session = openraveros_getglobalsession();
req = openraveros_env_getbodies();
req.bodyid = bodyid;
req.options = openraveros_BodyInfo().Req_Links();
res = rosoct_session_call(session.id,'env_getbodies',req);
if( ~isempty(res) )

    links = zeros(12,length(res.bodies{1}.links));
    for i = 1:size(links,2)
        links(:,i) = res.bodies{1}.links{i}.m;
    end
else
    links = [];
end
