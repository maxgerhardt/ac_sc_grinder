Import('env')
env.Append(LINKFLAGS=[ "--specs=rdimon.specs", "-lrdimon" ])
