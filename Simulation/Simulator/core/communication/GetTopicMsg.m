%  msg.status = -1;
%  
%  if isfield(com_data, 'topics')
%      for i=1:size(com_data.topics,2)
%          if strcmp(topic,com_data.topics(i).str)
%              msg = com_data.msg(i);
%              com_data.msg(i).status = 0;
%              break;
%          end
%      end
%  end
%