import os

from .annotations_test import anti_instagram_annotations_test

import duckietown_utils as dtu


@dtu.unit_test
def anti_instagram_annotations_test():
    out_base = dtu.get_output_dir_for_test()
    
#     out_base = 'anti_instagram_annotations_test'
    
    zipname = dtu.require_resource('ii-datasets.zip')
    dirname = os.path.dirname(zipname)
    base = os.path.join(dirname, 'ii-datasets')

    cmd = ['unzip', '-o', zipname]
    cwd = dirname
    dtu.system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)
    
    
    if not os.path.exists(base):
        msg = 'Could not find expected unzipped directory:\n   %s' % base
        raise Exception(msg)
    
    anti_instagram_annotations_test(base, out_base)
    
    

if __name__ == '__main__':
    dtu.run_tests_for_this_module()