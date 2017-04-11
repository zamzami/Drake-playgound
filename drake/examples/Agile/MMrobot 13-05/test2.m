zdim=5
cstr_name={'fun'}

      %for i=1:zdim 
           cstr_name_2=strcat(cstr_name,{sprintf('zdim2[%d]',i)});
           A=cellfun(@(a)[strcat(cstr_name,'nl_',num2str(a))],num2cell(1:zdim),'UniformOutput',false)
           %A3=arrayfun(@(a)[strcat(cstr_name,'nl_',num2str(a))],num2cell(1:zdim),'UniformOutput',false)
           A2=repmat(strcat(cstr_name,'nl_'),zdim,1)
      %end 